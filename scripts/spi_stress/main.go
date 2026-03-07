// SPI stress test sender for long-duration soak testing.
//
// Sends frames continuously at a target FPS and SPI clock, logs all events,
// and captures firmware serial output simultaneously.
//
// Run on the Raspberry Pi connected to the SPI slave board.
// Cross-compile: GOOS=linux GOARCH=arm64 go build -o spi_stress
package main

import (
	"bufio"
	"context"
	"encoding/binary"
	"flag"
	"fmt"
	"math"
	"math/rand"
	"os"
	"os/signal"
	"path/filepath"
"strconv"
	"strings"
	"syscall"
	"time"
	"unsafe"
)

// SPI constants
const (
	spiMode3 = 3 // CPOL=1, CPHA=1

	// ioctl commands for SPI
	spiIOCWrMode     = 0x40016B01
	spiIOCWrMaxSpeed = 0x40046B04
	spiIOCRdMaxSpeed = 0x80046B04
	spiIOCMessage1   = 0x40206B00 // SPI_IOC_MESSAGE(1)
)

// Speed sweep tiers (Hz)
var speedTiers = []int{5_000_000, 7_000_000, 10_000_000, 12_000_000, 15_000_000}

const sweepIntervalS = 1800 // 30 minutes per tier

// spiIOCTransfer matches the kernel's struct spi_ioc_transfer (64-bit)
type spiIOCTransfer struct {
	txBuf       uint64
	rxBuf       uint64
	len         uint32
	speedHz     uint32
	delayUsecs  uint16
	bitsPerWord uint8
	csChange    uint8
	txNbits     uint8
	rxNbits     uint8
	wordDelay   uint8
	pad         uint8
}

// CRC-16-CCITT lookup table (poly 0x1021, init 0xFFFF)
var crcTable [256]uint16

func init() {
	for i := 0; i < 256; i++ {
		crc := uint16(i) << 8
		for j := 0; j < 8; j++ {
			if crc&0x8000 != 0 {
				crc = (crc << 1) ^ 0x1021
			} else {
				crc <<= 1
			}
		}
		crcTable[i] = crc
	}
}

func crc16CCITT(data []byte) uint16 {
	crc := uint16(0xFFFF)
	for _, b := range data {
		crc = (crc << 8) ^ crcTable[byte(crc>>8)^b]
	}
	return crc
}

// buildFrameInto fills buf with a v2 framed packet and returns the total length.
// Protocol v2: sync(2) + length(4) + frame_num(4) + flags(1) + port_mask(4) + data(N) + crc(2)
func buildFrameInto(buf []byte, frameNum uint32, portMask uint32, pixelData []byte) int {
	dataLen := len(pixelData)
	// sync
	buf[0] = 0xAA
	buf[1] = 0x55
	// length (big-endian)
	binary.BigEndian.PutUint32(buf[2:6], uint32(dataLen))
	// frame number (big-endian)
	binary.BigEndian.PutUint32(buf[6:10], frameNum)
	// flags: upper nibble = protocol version 2, bit 0 = DMX (0 here)
	buf[10] = 2 << 4
	// port mask (big-endian)
	binary.BigEndian.PutUint32(buf[11:15], portMask)
	// pixel data
	copy(buf[15:], pixelData)
	// CRC over bytes 2..end (length + frame_num + flags + port_mask + pixel_data = 13 + dataLen)
	crc := crc16CCITT(buf[2 : 15+dataLen])
	end := 15 + dataLen
	buf[end] = byte(crc >> 8)
	buf[end+1] = byte(crc)
	return end + 2
}

// --- GPIO via /dev/gpiomem (mmap, BCM2711 Pi 4) ---
//
// Reads BCM2711 GPLEV registers directly, same as RPi.GPIO.
// Does not require exclusive line ownership — works alongside signald.

const gplevOffset = 0x34 // GPLEV0: GPIO pin level register (GPIO 0-31)

type mmapGPIO struct {
	mem  []byte
	reg  int // byte offset of GPLEV register (0x34 or 0x38)
	mask uint32
}

func gpioOpen(pin int) (*mmapGPIO, error) {
	f, err := os.OpenFile("/dev/gpiomem", os.O_RDONLY|os.O_SYNC, 0)
	if err != nil {
		return nil, fmt.Errorf("open /dev/gpiomem: %w", err)
	}
	defer f.Close()

	mem, err := syscall.Mmap(int(f.Fd()), 0, 4096, syscall.PROT_READ, syscall.MAP_SHARED)
	if err != nil {
		return nil, fmt.Errorf("mmap /dev/gpiomem: %w", err)
	}

	reg := gplevOffset
	if pin >= 32 {
		reg += 4 // GPLEV1 for GPIO 32-53
	}

	return &mmapGPIO{
		mem:  mem,
		reg:  reg,
		mask: 1 << (uint(pin) % 32),
	}, nil
}

func (g *mmapGPIO) Read() bool {
	level := *(*uint32)(unsafe.Pointer(&g.mem[g.reg]))
	return (level & g.mask) != 0
}

// ReadRaw returns the full GPLEV register value (all 32 GPIO pins)
func (g *mmapGPIO) ReadRaw() uint32 {
	return *(*uint32)(unsafe.Pointer(&g.mem[g.reg]))
}

func (g *mmapGPIO) Close() {
	syscall.Munmap(g.mem)
}

// waitReady waits for the READY pin to go HIGH. Returns false on timeout or context cancellation.
func waitReady(ctx context.Context, gpio *mmapGPIO, timeout time.Duration) bool {
	deadline := time.Now().Add(timeout)
	for !gpio.Read() {
		if time.Now().After(deadline) {
			return false
		}
		select {
		case <-ctx.Done():
			return false
		default:
		}
		time.Sleep(50 * time.Microsecond)
	}
	return true
}

// --- SPI via ioctl ---

type spiDev struct {
	fd    int
	speed int
}

func spiOpen(device string, speed int) (*spiDev, error) {
	fd, err := syscall.Open(device, syscall.O_RDWR, 0)
	if err != nil {
		return nil, fmt.Errorf("spi open %s: %w", device, err)
	}

	s := &spiDev{fd: fd, speed: speed}

	// Set mode 3
	mode := uint8(spiMode3)
	if err := s.ioctl(spiIOCWrMode, uintptr(unsafe.Pointer(&mode))); err != nil {
		syscall.Close(fd)
		return nil, fmt.Errorf("spi set mode: %w", err)
	}

	// Set speed
	if err := s.setSpeed(speed); err != nil {
		syscall.Close(fd)
		return nil, err
	}

	return s, nil
}

func (s *spiDev) ioctl(req uintptr, arg uintptr) error {
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, uintptr(s.fd), req, arg)
	if errno != 0 {
		return errno
	}
	return nil
}

func (s *spiDev) setSpeed(hz int) error {
	speed := uint32(hz)
	if err := s.ioctl(spiIOCWrMaxSpeed, uintptr(unsafe.Pointer(&speed))); err != nil {
		return fmt.Errorf("spi set speed: %w", err)
	}
	s.speed = hz
	return nil
}

// transfer sends data in a single CS cycle via SPI_IOC_MESSAGE(1).
func (s *spiDev) transfer(data []byte) error {
	xfer := spiIOCTransfer{
		txBuf:   uint64(uintptr(unsafe.Pointer(&data[0]))),
		len:     uint32(len(data)),
		speedHz: uint32(s.speed),
	}
	return s.ioctl(spiIOCMessage1, uintptr(unsafe.Pointer(&xfer)))
}

func (s *spiDev) Close() {
	syscall.Close(s.fd)
}

// --- Pixel patterns ---

type pattern struct {
	name string
	data []byte
}

func generatePixelPatterns(numPorts, pixelsPerPort int, brightness float64) []pattern {
	totalPx := numPorts * pixelsPerPort
	n := totalPx * 3

	scale := func(r, g, b int) (byte, byte, byte) {
		return byte(float64(r) * brightness),
			byte(float64(g) * brightness),
			byte(float64(b) * brightness)
	}

	solid := func(name string, r, g, b int) pattern {
		sr, sg, sb := scale(r, g, b)
		data := make([]byte, n)
		for i := 0; i < totalPx; i++ {
			data[i*3] = sr
			data[i*3+1] = sg
			data[i*3+2] = sb
		}
		return pattern{name: name, data: data}
	}

	patterns := []pattern{
		solid("red", 255, 0, 0),
		solid("green", 0, 255, 0),
		solid("blue", 0, 0, 255),
		solid("white", 255, 255, 255),
	}

	// gradient
	gradData := make([]byte, n)
	for port := 0; port < numPorts; port++ {
		base := port * pixelsPerPort * 3
		for px := 0; px < pixelsPerPort; px++ {
			denom := pixelsPerPort - 1
			if denom < 1 {
				denom = 1
			}
			v := (px * 255) / denom
			off := base + px*3
			gradData[off] = byte(float64(v) * brightness)
			gradData[off+1] = byte(float64(255-v) * brightness)
			gradData[off+2] = byte(float64((v*3)&0xFF) * brightness)
		}
	}
	patterns = append(patterns, pattern{name: "gradient", data: gradData})

	// random
	randData := make([]byte, n)
	rand.Read(randData)
	for i := range randData {
		randData[i] = byte(float64(randData[i]) * brightness)
	}
	patterns = append(patterns, pattern{name: "random", data: randData})

	return patterns
}

// --- Serial port (termios) ---

// Minimal termios constants for linux/arm64
const (
	tcgets2 = 0x802C542A // _IOR('T', 0x2A, struct termios2)
	tcsets2 = 0x402C542B // _IOW('T', 0x2B, struct termios2)
	bother  = 0x1000
	clocal  = 0x800
	cread   = 0x80
	cs8     = 0x30
	vmin    = 6
	vtime   = 5
)

// termios2 matches the kernel's struct termios2 (44 bytes on arm64)
type termios2 struct {
	cIflag  uint32
	cOflag  uint32
	cCflag  uint32
	cLflag  uint32
	cLine   uint8
	cCc     [19]uint8
	cIspeed uint32
	cOspeed uint32
}

func openSerial(path string, baud int) (*os.File, error) {
	f, err := os.OpenFile(path, os.O_RDWR|syscall.O_NOCTTY|syscall.O_NONBLOCK, 0)
	if err != nil {
		return nil, err
	}

	fd := f.Fd()

	// Get current termios
	var tio termios2
	_, _, errno := syscall.Syscall(syscall.SYS_IOCTL, fd, tcgets2, uintptr(unsafe.Pointer(&tio)))
	if errno != 0 {
		f.Close()
		return nil, fmt.Errorf("tcgets2: %w", errno)
	}

	// Raw mode: clear all input/output/local processing
	tio.cIflag = 0
	tio.cOflag = 0
	tio.cLflag = 0
	tio.cCflag = bother | cs8 | clocal | cread
	tio.cIspeed = uint32(baud)
	tio.cOspeed = uint32(baud)
	tio.cCc[vmin] = 1  // block until at least 1 byte
	tio.cCc[vtime] = 1 // 100ms timeout

	_, _, errno = syscall.Syscall(syscall.SYS_IOCTL, fd, tcsets2, uintptr(unsafe.Pointer(&tio)))
	if errno != 0 {
		f.Close()
		return nil, fmt.Errorf("tcsets2: %w", errno)
	}

	// Clear O_NONBLOCK now that termios is set
	flags, _, _ := syscall.Syscall(syscall.SYS_FCNTL, fd, syscall.F_GETFL, 0)
	syscall.Syscall(syscall.SYS_FCNTL, fd, syscall.F_SETFL, flags & ^uintptr(syscall.O_NONBLOCK))

	return f, nil
}

// --- Serial capture ---

func serialCapture(ctx context.Context, serialPort, logFile string) {
	f, err := openSerial(serialPort, 115200)
	if err != nil {
		fmt.Printf("  serial: could not open %s: %v\n", serialPort, err)
		return
	}
	defer f.Close()

	// close the fd when context cancels to unblock the blocking read
	go func() {
		<-ctx.Done()
		f.Close()
	}()

	out, err := os.Create(logFile)
	if err != nil {
		fmt.Printf("  serial: could not create %s: %v\n", logFile, err)
		return
	}
	defer out.Close()

	scanner := bufio.NewScanner(f)
	for scanner.Scan() {
		ts := time.Now().Format("15:04:05")
		fmt.Fprintf(out, "[%s] %s\n", ts, scanner.Text())
	}
}

// --- Helpers ---

func formatDuration(seconds float64) string {
	s := int(seconds)
	h := s / 3600
	m := (s % 3600) / 60
	sec := s % 60
	return fmt.Sprintf("%dh %02dm %02ds", h, m, sec)
}

func roundTo(v float64, decimals int) float64 {
	p := math.Pow(10, float64(decimals))
	return math.Round(v*p) / p
}

// isFlagSet returns true if a flag was explicitly provided on the command line.
func isFlagSet(name string) bool {
	found := false
	flag.Visit(func(f *flag.Flag) {
		if f.Name == name {
			found = true
		}
	})
	return found
}

// --- Main ---

func main() {
	speed := flag.Int("s", 15_000_000, "SPI clock Hz")
	pixels := flag.Int("n", 300, "pixels per port")
	ports := flag.Int("p", 8, "number of ports")
	fps := flag.Int("f", 40, "target frame rate")
	duration := flag.Int("duration", 28800, "test duration seconds")
	serialPort := flag.String("serial", "/dev/ttyACM0", "firmware serial port")
	logDir := flag.String("log-dir", "./stress_logs", "output directory for logs")
	speedSweep := flag.Bool("speed-sweep", false, "ramp SPI speed every 30min")
	readyPin := flag.Int("ready-pin", 16, "BCM GPIO pin for READY")
	fpsSweepStr := flag.String("fps-sweep", "", "FPS sweep range, e.g. 35-45")
	fpsStep := flag.Int("fps-step", 120, "seconds per FPS step during sweep")
	debug := flag.Bool("debug", false, "enable timing diagnostics on timeout")
	flag.Parse()

	// Parse FPS sweep range
	var fpsSweepStart, fpsSweepEnd int
	fpsSweepEnabled := false
	if *fpsSweepStr != "" {
		parts := strings.SplitN(*fpsSweepStr, "-", 2)
		if len(parts) != 2 {
			fmt.Fprintf(os.Stderr, "error: --fps-sweep must be START-END, e.g. 35-45\n")
			os.Exit(1)
		}
		var err1, err2 error
		fpsSweepStart, err1 = strconv.Atoi(parts[0])
		fpsSweepEnd, err2 = strconv.Atoi(parts[1])
		if err1 != nil || err2 != nil || fpsSweepStart < 1 || fpsSweepEnd < fpsSweepStart {
			fmt.Fprintf(os.Stderr, "error: --fps-sweep invalid range %q\n", *fpsSweepStr)
			os.Exit(1)
		}
		fpsSweepEnabled = true
		*fps = fpsSweepStart
		// Auto-set duration to cover entire sweep if user didn't override
		sweepDuration := (fpsSweepEnd - fpsSweepStart + 1) * *fpsStep
		if !isFlagSet("duration") {
			*duration = sweepDuration
		}
	}

	// Create timestamped log directory
	runName := time.Now().Format("2006-01-02_150405")
	logPath := filepath.Join(*logDir, runName)
	if err := os.MkdirAll(logPath, 0755); err != nil {
		fmt.Fprintf(os.Stderr, "error creating log dir: %v\n", err)
		os.Exit(1)
	}

	senderLog := filepath.Join(logPath, "pi_sender.jsonl")
	serialLog := filepath.Join(logPath, "firmware_serial.log")

	// GPIO setup
	gpio, err := gpioOpen(*readyPin)
	if err != nil {
		fmt.Fprintf(os.Stderr, "error: %v\n", err)
		os.Exit(1)
	}
	defer gpio.Close()

	// SPI setup
	spi, err := spiOpen("/dev/spidev0.0", *speed)
	if err != nil {
		fmt.Fprintf(os.Stderr, "error: %v\n", err)
		os.Exit(1)
	}
	defer spi.Close()

	// Signal handling: first Ctrl+C triggers graceful shutdown, second forces exit
	mainCtx, mainCancel := context.WithCancel(context.Background())
	sigCh := make(chan os.Signal, 2)
	signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)
	go func() {
		<-sigCh
		fmt.Println("\n  shutting down...")
		mainCancel()
		<-sigCh
		fmt.Println("\n  forced exit")
		os.Exit(1)
	}()

	// Start serial capture (child of mainCtx so Ctrl+C stops it too)
	serialCtx, serialCancel := context.WithCancel(mainCtx)
	_ = serialCancel // called implicitly via mainCancel; process exit cleans up
	go serialCapture(serialCtx, *serialPort, serialLog)

	// Stats
	currentSpeed := *speed
	var totalFrames uint32
	var readyTimeouts int
	startTime := time.Now()
	nextSummaryTime := startTime.Add(60 * time.Second)
	sweepStartTime := startTime
	tierIndex := 0

	// Per-step FPS tracking (reset at each FPS sweep step)
	stepStartTime := startTime
	var stepFrames uint32

	framePeriod := time.Duration(float64(time.Second) / float64(*fps))
	bytesPerFrame := *ports * *pixels * 3
	pktSize := bytesPerFrame + 17 // v2 header(15) + CRC(2)

	// Check spidev bufsiz
	bufsizPath := "/sys/module/spidev/parameters/bufsiz"
	if data, err := os.ReadFile(bufsizPath); err == nil {
		if bufsiz, err := strconv.Atoi(strings.TrimSpace(string(data))); err == nil {
			if bufsiz < pktSize {
				needed := pktSize
				if needed < 65536 {
					needed = 65536
				}
				fmt.Printf("WARNING: spidev bufsiz=%d < frame size %d\n", bufsiz, pktSize)
				fmt.Printf("  ioctl path doesn't split, but consider: sudo sh -c 'echo %d > %s'\n", needed, bufsizPath)
			}
		}
	}

	sweepStr := "OFF"
	if *speedSweep {
		parts := make([]string, len(speedTiers))
		for i, s := range speedTiers {
			parts[i] = fmt.Sprintf("%.0fM", float64(s)/1e6)
		}
		sweepStr = "ON (" + strings.Join(parts, ", ") + ")"
	}

	fpsSweepStr2 := "OFF"
	if fpsSweepEnabled {
		fpsSweepStr2 = fmt.Sprintf("ON (%d-%d fps, %ds/step)", fpsSweepStart, fpsSweepEnd, *fpsStep)
	}
	currentFPS := *fps

	// Write run parameters to params.json for the report script
	paramsJSON := fmt.Sprintf(
		`{"spi_mhz":%.1f,"ports":%d,"pixels":%d,"bytes_per_frame":%d,"target_fps":%d,"duration_s":%d,"speed_sweep":%v,"fps_sweep":%q,"fps_step":%d,"ready_pin":%d,"serial_port":%q}`+"\n",
		float64(currentSpeed)/1e6, *ports, *pixels, bytesPerFrame, *fps, *duration, *speedSweep, *fpsSweepStr, *fpsStep, *readyPin, *serialPort)
	os.WriteFile(filepath.Join(logPath, "params.json"), []byte(paramsJSON), 0644)

	fmt.Println("=== SPI STRESS TEST ===")
	fmt.Printf("SPI:      %.1f MHz, Mode 3\n", float64(currentSpeed)/1e6)
	fmt.Printf("Config:   %d ports x %d px = %d bytes/frame\n", *ports, *pixels, bytesPerFrame)
	fmt.Printf("Target:   %d fps for %s\n", *fps, formatDuration(float64(*duration)))
	fmt.Printf("Sweep:    %s\n", sweepStr)
	fmt.Printf("FPS:      %s\n", fpsSweepStr2)
	fmt.Printf("Logs:     %s/\n", logPath)
	fmt.Println()

	// Wait for initial READY
	fmt.Print("waiting for READY...")
	if !waitReady(mainCtx, gpio, 10*time.Second) {
		fmt.Println(" timeout! check wiring and firmware.")
		os.Exit(1)
	}
	fmt.Println(" ok")

	// Send alignment frame (4 zero bytes)
	alignBuf := make([]byte, 4)
	if err := spi.transfer(alignBuf); err != nil {
		fmt.Fprintf(os.Stderr, "  sync: alignment frame failed: %v\n", err)
	} else {
		fmt.Println("  sync: 4-byte alignment frame sent")
	}
	time.Sleep(500 * time.Millisecond)

	// Port mask: all ports active (bits 0..ports-1 set)
	portMask := uint32((1 << *ports) - 1)

	// Pre-generate pixel patterns and frame buffer
	patterns := generatePixelPatterns(*ports, *pixels, 0.3)
	frameBuf := make([]byte, pktSize)
	numPatterns := len(patterns)

	fmt.Printf("  streaming at %d fps...\n\n", currentFPS)

	// Open log file
	logFile, err := os.Create(senderLog)
	if err != nil {
		fmt.Fprintf(os.Stderr, "error creating log: %v\n", err)
		os.Exit(1)
	}
	logWriter := bufio.NewWriterSize(logFile, 64*1024)

	testDeadline := startTime.Add(time.Duration(*duration) * time.Second)

	// debug timing: track timestamps from previous iteration to diagnose stalls
	var dbgLastIterEnd time.Time       // when the previous loop iteration ended
	var dbgLastSPIDone time.Time       // when the previous SPI transfer completed
	var dbgLastPaceSleep time.Duration // how long the previous pacing sleep was requested
	var dbgLastTimeoutFrame uint32     // frame number of last logged timeout_debug

	// accumulate ALL log entries in memory — zero I/O in hot path.
	// fmt.Fprintf to a file-backed bufio.Writer triggers SD card flushes
	// (10-50ms) that cause READY timeouts. dump everything at end instead.
	type frameLog struct {
		t         float64
		frame     uint32
		bytes     int
		readyMs   float64
		spiMhz    float64
		targetFPS int
		isTimeout bool
	}
	frameLogs := make([]frameLog, 0, *fps**duration+1000)

	// debug: accumulate timeout diagnostics in memory, dump at end
	type timeoutDiag struct {
		t             float64
		frame         uint32
		gpioAtEntry   bool
		rawGPLEV      uint32  // full GPLEV0 register at timeout
		sinceIterEnd  float64 // ms
		sinceSPIDone  float64 // ms
		loopTop       float64 // ms
		prevPaceReq   float64 // ms
	}
	var dbgDiags []timeoutDiag

	for mainCtx.Err() == nil {
		// Check duration
		if time.Now().After(testDeadline) {
			break
		}

		// Speed sweep
		if *speedSweep {
			sweepElapsed := time.Since(sweepStartTime).Seconds()
			newTier := int(sweepElapsed / float64(sweepIntervalS))
			if newTier >= len(speedTiers) {
				newTier = len(speedTiers) - 1
			}
			if newTier != tierIndex {
				tierIndex = newTier
				currentSpeed = speedTiers[tierIndex]
				spi.setSpeed(currentSpeed)
				fmt.Printf("  [sweep] SPI speed -> %.0f MHz\n", float64(currentSpeed)/1e6)
			}
		}

		// Periodic console summary (every 60s, anchored to startTime to avoid drift)
		if stepFrames > 0 && time.Now().After(nextSummaryTime) {
			now := time.Now()
			stepElapsed := now.Sub(stepStartTime).Seconds()
			stepFPS := float64(stepFrames) / stepElapsed
			ts := formatDuration(now.Sub(startTime).Seconds())
			fmt.Printf("  [%s] sent:%d ready_timeout:%d fps:%.1f target_fps:%d spi:%.0fMHz\n",
				ts, totalFrames, readyTimeouts, stepFPS, currentFPS, float64(currentSpeed)/1e6)
			nextSummaryTime = nextSummaryTime.Add(60 * time.Second)
		}

		// FPS sweep
		if fpsSweepEnabled {
			elapsed := time.Since(startTime).Seconds()
			step := int(elapsed / float64(*fpsStep))
			newFPS := fpsSweepStart + step
			if newFPS > fpsSweepEnd {
				break // sweep complete
			}
			if newFPS != currentFPS {
				currentFPS = newFPS
				framePeriod = time.Duration(float64(time.Second) / float64(currentFPS))
				stepStartTime = time.Now()
				stepFrames = 0
				fmt.Printf("  [fps-sweep] %d fps (step %d/%d)\n",
					currentFPS, step+1, fpsSweepEnd-fpsSweepStart+1)
			}
		}

		frameStart := time.Now()

		// Wait for READY
		readyStart := time.Now()
		gpioAtEntry := gpio.Read() // capture raw GPIO state before polling
		if !waitReady(mainCtx, gpio, 20*time.Millisecond) {
			readyTimeouts++
			t := roundTo(time.Since(startTime).Seconds(), 3)
			spiMhz := roundTo(float64(currentSpeed)/1e6, 1)
			frameLogs = append(frameLogs, frameLog{
				t: t, frame: totalFrames + 1, spiMhz: spiMhz,
				targetFPS: currentFPS, isTimeout: true,
			})

			// debug: capture timing context in memory (zero I/O in hot path)
			if *debug && (readyTimeouts == 1 || totalFrames+1 != dbgLastTimeoutFrame) {
				dbgLastTimeoutFrame = totalFrames + 1
				sinceIterEnd := float64(0)
				if !dbgLastIterEnd.IsZero() {
					sinceIterEnd = time.Since(dbgLastIterEnd).Seconds() * 1000
				}
				sinceSPIDone := float64(0)
				if !dbgLastSPIDone.IsZero() {
					sinceSPIDone = time.Since(dbgLastSPIDone).Seconds() * 1000
				}
				loopTop := readyStart.Sub(frameStart).Seconds() * 1000
				dbgDiags = append(dbgDiags, timeoutDiag{
					t:            t,
					frame:        totalFrames + 1,
					gpioAtEntry:  gpioAtEntry,
					rawGPLEV:     gpio.ReadRaw(),
					sinceIterEnd: sinceIterEnd,
					sinceSPIDone: sinceSPIDone,
					loopTop:      loopTop,
					prevPaceReq:  float64(dbgLastPaceSleep) / float64(time.Millisecond),
				})
			}
			continue
		}
		readyMs := time.Since(readyStart).Seconds() * 1000

		totalFrames++
		stepFrames++
		pixelData := patterns[int(totalFrames)%numPatterns].data
		pktLen := buildFrameInto(frameBuf, totalFrames, portMask, pixelData)

		if err := spi.transfer(frameBuf[:pktLen]); err != nil {
			fmt.Fprintf(os.Stderr, "  spi transfer error: %v\n", err)
		}
		dbgLastSPIDone = time.Now()

		// Log frame (in memory — no I/O in hot path)
		t := roundTo(time.Since(startTime).Seconds(), 3)
		rMs := roundTo(readyMs, 2)
		spiMhz := roundTo(float64(currentSpeed)/1e6, 1)
		frameLogs = append(frameLogs, frameLog{
			t: t, frame: totalFrames, bytes: pktLen,
			readyMs: rMs, spiMhz: spiMhz, targetFPS: currentFPS,
		})

		// Pace to target FPS: sleep most of the remaining time, busy-wait the last 500us
		frameElapsed := time.Since(frameStart)
		remaining := framePeriod - frameElapsed
		dbgLastPaceSleep = 0
		if remaining > 500*time.Microsecond {
			dbgLastPaceSleep = remaining - 500*time.Microsecond
			time.Sleep(dbgLastPaceSleep)
		}
		// Busy-wait for precision (check context to allow Ctrl+C escape)
		for time.Since(frameStart) < framePeriod && mainCtx.Err() == nil {
			// spin
		}
		dbgLastIterEnd = time.Now()
	}

	// Write all accumulated log entries to disk (deferred from hot path)
	fmt.Printf("  writing %d log entries...\n", len(frameLogs))
	for _, fl := range frameLogs {
		if fl.isTimeout {
			fmt.Fprintf(logWriter, `{"t":%.3f,"frame":%d,"event":"ready_timeout","spi_mhz":%.1f}`+"\n",
				fl.t, fl.frame, fl.spiMhz)
		} else {
			fmt.Fprintf(logWriter, `{"t":%.3f,"frame":%d,"bytes":%d,"ready_ms":%.2f,"spi_mhz":%.1f,"target_fps":%d}`+"\n",
				fl.t, fl.frame, fl.bytes, fl.readyMs, fl.spiMhz, fl.targetFPS)
		}
	}

	// Write debug diagnostics
	if *debug && len(dbgDiags) > 0 {
		for _, d := range dbgDiags {
			fmt.Fprintf(logWriter,
				`{"t":%.3f,"frame":%d,"event":"timeout_debug","gpio_at_entry":%v,`+
					`"raw_gplev":"0x%08x","ready_bit":%v,`+
					`"since_iter_end_ms":%.2f,"since_spi_done_ms":%.2f,`+
					`"loop_top_ms":%.2f,"prev_pace_req_ms":%.2f}`+"\n",
				d.t, d.frame, d.gpioAtEntry,
				d.rawGPLEV, (d.rawGPLEV>>16)&1 == 1,
				d.sinceIterEnd, d.sinceSPIDone,
				d.loopTop, d.prevPaceReq)
		}
		fmt.Printf("  debug: %d timeout diagnostics written to log\n", len(dbgDiags))
	}

	// Flush and close log
	logWriter.Flush()
	logFile.Close()

	// Final summary
	dur := time.Since(startTime).Seconds()
	avgFPS := float64(totalFrames) / dur
	totalData := int(totalFrames) * pktSize

	fmt.Println()
	fmt.Println("=== STRESS TEST COMPLETE ===")
	fmt.Printf("Duration:       %s\n", formatDuration(dur))
	fmt.Printf("Frames sent:    %d\n", totalFrames)
	fmt.Printf("READY timeouts: %d\n", readyTimeouts)
	fmt.Printf("Avg FPS:        %.1f\n", avgFPS)
	fmt.Printf("SPI speed:      %.0f MHz\n", float64(currentSpeed)/1e6)
	fmt.Printf("Total bytes:    %d\n", totalData)
	fmt.Printf("Log files:      %s/\n", logPath)

	// Exit immediately -- serial capture goroutine uses blocking I/O
	// that can't be reliably interrupted, so let process exit clean it up.
	os.Exit(0)
}
