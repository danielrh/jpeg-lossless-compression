/*  Sirikata Jpeg Texture Transfer Compression -- Texture Transfer management system
 *  reader.go
 *
 *  Copyright (c) 2015, Daniel Reiter Horn
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Copyright 2009 The Go Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Package jpeg implements a JPEG image decoder and encoder.
//
// JPEG is defined in ITU-T T.81: http://www.w3.org/Graphics/JPEG/itu-t81.pdf.
//
// Port from Sirikata C++ to golang by Daniel Reiter Horn
//
package main

import (
	"errors"
	"fmt"
	"io"
	"log"
)

// TODO(nigeltao): fix up the doc comment style so that sentences start with
// the name of the type or function that they annotate.

// A FormatError reports that the input is not a valid JPEG.
type FormatError string

func (e FormatError) Error() string { return "invalid JPEG format: " + string(e) }

// An UnsupportedError reports that the input uses a valid but unimplemented JPEG feature.
type UnsupportedError string

func (e UnsupportedError) Error() string { return "unsupported JPEG feature: " + string(e) }

const blockSize = 64 // A DCT block is 8x8.

type block [blockSize]int32

// Component specification, specified in section B.2.2.
type component struct {
	h  int   // Horizontal sampling factor.
	v  int   // Vertical sampling factor.
	c  uint8 // Component identifier.
	tq uint8 // Quantization table destination selector.
}

const (
	dcTable = 0
	acTable = 1
	maxTc   = 1
	maxTh   = 3
	maxTq   = 3

	// A grayscale JPEG image has only a Y component.
	nGrayComponent = 1
	// A color JPEG image has Y, Cb and Cr components.
	nColorComponent = 3

	// We only support 4:4:4, 4:4:0, 4:2:2 and 4:2:0 downsampling, and therefore the
	// number of luma samples per chroma sample is at most 2 in the horizontal
	// and 2 in the vertical direction.
	maxH = 2
	maxV = 2
)

const (
	soiMarker   = 0xd8 // Start Of Image.
	eoiMarker   = 0xd9 // End Of Image.
	sof0Marker  = 0xc0 // Start Of Frame (Baseline).
	sof2Marker  = 0xc2 // Start Of Frame (Progressive).
	dhtMarker   = 0xc4 // Define Huffman Table.
	dqtMarker   = 0xdb // Define Quantization Table.
	sosMarker   = 0xda // Start Of Scan.
	driMarker   = 0xdd // Define Restart Interval.
	rst0Marker  = 0xd0 // ReSTart (0).
	rst7Marker  = 0xd7 // ReSTart (7).
	app0Marker  = 0xe0 // APPlication specific (0).
	app15Marker = 0xef // APPlication specific (15).
	comMarker   = 0xfe // COMment.
)

// unzig maps from the zig-zag ordering to the natural ordering. For example,
// unzig[3] is the column and row of the fourth element in zig-zag order. The
// value is 16, which means first column (16%8 == 0) and third row (16/8 == 2).
var unzig = [blockSize]int{
	0, 1, 8, 16, 9, 2, 3, 10,
	17, 24, 32, 25, 18, 11, 4, 5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13, 6, 7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47, 55, 62, 63,
}

// Reader is deprecated.
type Reader interface {
	io.ByteReader
	io.Reader
}

// bits holds the unprocessed bits that have been taken from the byte-stream.
// The n least significant bits of a form the unread bits, to be read in MSB to
// LSB order.
type bits struct {
	a uint32 // accumulator.
	m uint32 // mask. m==1<<(n-1) when n>0, with m==0 when n==0.
	n int32  // the number of unread bits in a.
}

const (
	optionReserved0 = 1 << iota
	optionReserved1 = 1 << iota
	optionReserved2 = 1 << iota
	optionReserved3 = 1 << iota
	optionReserved4 = 1 << iota
	comp01coalesce  = 1 << iota
	comp02coalesce  = 1 << iota
	comp12coalesce  = 1 << iota
)

type decoder struct {
	r    io.Reader
	bits bits
	// bytes is a byte buffer, similar to a bufio.Reader, except that it
	// has to be able to unread more than 1 byte, due to byte stuffing.
	// Byte stuffing is specified in section F.1.2.3.
	bytes struct {
		// buf[i:j] are the buffered bytes read from the underlying
		// io.Reader that haven't yet been passed further on.
		buf  [4096]byte
		i, j int
		// nUnreadable is the number of bytes to back up i after
		// overshooting. It can be 0, 1 or 2.
		nUnreadable int
	}
	width, height int
	ri            int // Restart Interval.
	nComp         int
	progressive   bool
	arhc          bool   // is this in compressed arhc format
	offset        uint32 // where we are in the file (for debug purposes)
	huffTransSect bool   // are we in the huffman translation section of code
	eobRun        uint16 // End-of-Band run, specified in section G.1.2.2.
	comp          [nColorComponent]component
	progCoeffs    [nColorComponent][]block // Saved state between progressive-mode scans.
	huff          [maxTc + 1][maxTh + 1]huffman
	quant         [maxTq + 1]block // Quantization tables, in zig-zag order.
	tmp           [blockSize + 1]byte
	wbuffer       BitByteStream
	bitbuffer     BitByteStream // the buffer of bitstreams at the beginning of file
	// this is used to reconstruct the scan regions without
	// clogging up the parts that should be huffman coded
	huffMultibuffer     []BitByteStream // one stream per 64 value per channel
	componentCoalescing uint8           // & 0x20 if comp[0] and comp[1] are coalesced
	// & 0x40 if comp[1] and comp[2] are coalesced
	// & 0x80 if comp[0] and comp[2] are coalesced
	extOriginalFileSize    uint32
	extEndFileBuffer       []byte
	extEndFileBufferCursor uint32
	extEndEncountered      bool
}

type BitByteStream struct {
	buffer        []uint8
	bits          uint32
	nBits         uint8
	bitReadCursor uint32
}

func (b *BitByteStream) appendByte(x *byte) {
	b.buffer = append(b.buffer, *x)
}

func (b *BitByteStream) appendBytes(x []byte) {
	b.buffer = append(b.buffer, x...)
}

func (b *BitByteStream) clear() {
	b.buffer = []byte{}
}

func (b *BitByteStream) flushToWriter(w io.Writer) {
	w.Write(b.buffer)
	b.buffer = []byte{}
}

func (b *BitByteStream) emitBits(bits, nBits uint32, stuffZeros bool) {
	nBits += uint32(b.nBits)
	bits <<= 32 - nBits
	bits |= b.bits
	for nBits >= 8 {
		bb := uint8(bits >> 24)
		b.appendByte(&bb)
		//fmt.Printf("Appending byte: %x\n", bb)
		if bb == 0xff && stuffZeros {
			var zero uint8
			//fmt.Printf("Appending stuffed zero: %x\n", zero)
			b.appendByte(&zero)
		}
		bits <<= 8
		nBits -= 8
	}
	//fmt.Printf("Leftovers %d bits %x\n", nBits, bits)
	b.bits, b.nBits = bits, uint8(nBits)
}

func (b *BitByteStream) scanBits(nBits uint32, stuffZeros bool) (uint32, error) {
	if nBits > 16 {
		return 0, errors.New(fmt.Sprintf("Must have nBits = %d < 16", nBits))
	}
	if nBits == 0 {
		return 0, nil // don't read off the array since it may be empty or at its end
	}
	byteAddress := b.bitReadCursor / 8
	if int(byteAddress) >= len(b.buffer) {
		return 0, errors.New(fmt.Sprintf("Reading[%d] off the end of len(%d) bit buffer",
			byteAddress, len(b.buffer)))
	}
	bitAddress := b.bitReadCursor - byteAddress*8
	var retval uint32
	curByte := b.buffer[byteAddress] & ((1 << (8 - bitAddress)) - 1)
	retval |= uint32(curByte)
	remainingBitsInByte := 8 - bitAddress
	//fmt.Printf("Retval %x[%d].%d so far,  Remaining bits %d\n", retval, byteAddress,bitAddress,nBits)
	if remainingBitsInByte <= nBits && b.buffer[byteAddress] == 0xff && stuffZeros {
		if b.buffer[byteAddress+1] != 0x0 {
			panic("Must be stuffed byte")
		}
		//fmt.Printf("Stuffed byte at %d\n", byteAddress)
		byteAddress++
		b.bitReadCursor += 8
	}
	if remainingBitsInByte >= nBits {
		retval >>= remainingBitsInByte - nBits
		//fmt.Printf("Returning early after single byte read\n")
		b.bitReadCursor += nBits
		return retval, nil
	}
	if int(byteAddress) >= len(b.buffer) {
		return 0, errors.New(fmt.Sprintf("Reading[%d] off the end of len(%d) bit buffer",
			byteAddress, len(b.buffer)))
	}
	b.bitReadCursor += remainingBitsInByte
	nBits -= remainingBitsInByte
	if nBits > 8 {
		b.bitReadCursor += 8
		byteAddress += 1
		retval <<= 8
		retval |= uint32(b.buffer[byteAddress])
		nBits -= 8
		if b.buffer[byteAddress] == 0xff && stuffZeros {
			//fmt.Printf("Second byte needs stuffing @ %d\n", byteAddress)
			if int(byteAddress)+1 >= len(b.buffer) {
				return 0, errors.New(fmt.Sprintf("Reading[%d] off the end of len(%d) bit buffer",
					byteAddress+1, len(b.buffer)))
			}
			if b.buffer[byteAddress+1] != 0x0 {
				return 0, errors.New(fmt.Sprintf("Reading[%d] should be stuffed with 0 not %d",
					byteAddress+1, b.buffer[byteAddress+1]))
			}
			byteAddress++
			b.bitReadCursor += 8
		}
	}

	if nBits > 8 {
		panic("unreachable: we should have only 16 bits to grab")
	}
	//fmt.Printf("Pref Final things are %x going to read %x after shifting %d\n", retval, b.buffer[byteAddress + 1], nBits)
	if int(byteAddress)+1 >= len(b.buffer) {
		return 0, errors.New(fmt.Sprintf("Reading[%d] off the end of len(%d) bit buffer",
			byteAddress+1, len(b.buffer)))
	}
	retval <<= nBits
	retval |= uint32(b.buffer[byteAddress+1] >> (8 - nBits))
	b.bitReadCursor += nBits
	//fmt.Printf("Final value %x\n", retval)
	if nBits == 8 && b.buffer[byteAddress+1] == 0xff && stuffZeros {
		if int(byteAddress)+2 >= len(b.buffer) {
			return 0, errors.New(fmt.Sprintf("Reading[%d] off the end of len(%d) bit buffer",
				byteAddress+2, len(b.buffer)))
		}
		if b.buffer[byteAddress+2] != 0x0 {
			return retval, errors.New(fmt.Sprintf("Reading[%d] should be stuffed with 0 not %d",
				byteAddress+2, b.buffer[byteAddress+2]))
		}
		byteAddress++
		b.bitReadCursor += 8
	}
	return retval, nil
}

func (b *BitByteStream) pop() {
	if b.nBits > 0 && b.nBits < 8 {
		var poppedByte uint32
		poppedByte = uint32(b.buffer[len(b.buffer)-1])
		b.buffer = b.buffer[0 : len(b.buffer)-1]
		poppedByte <<= b.nBits
		b.bits |= poppedByte
		b.nBits += 8
	}
	if b.nBits >= 8 {
		b.nBits -= 8
		b.bits >>= 8
	} else {
		b.buffer = b.buffer[0 : len(b.buffer)-1]
	}
}
func (b *BitByteStream) len() uint32 {
	return uint32(len(b.buffer)) + uint32(b.nBits/8)
}
func (b *BitByteStream) flushBits(stuffBits bool) {
	for b.nBits > 0 {
		b.emitBits(1, 1, stuffBits)
	}
}

func streamLenToBE(streamLen uint32) []byte {
	return []byte{uint8(streamLen >> 24), uint8((streamLen >> 16) & 0xff),
		uint8((streamLen >> 8) & 0xff), uint8(streamLen & 0xff)}
}
func bufferBEToStreamLength(buf []byte) uint32 {
	var vectorLength uint32
	for i := 0; i < 4; i++ { // read in the huffvector length
		vectorLength <<= 8
		vectorLength |= uint32(buf[i])
	}
	return vectorLength
}

var VERSION_INFORMATION = []byte{1, // major version
	0, // minor version
} // extensions follow
func (d *decoder) flush(w io.Writer) {
	if d.arhc {
		if d.wbuffer.nBits > 0 {
			panic("Bits should be flushed by here")
		}
		for len(d.wbuffer.buffer) < int(d.extOriginalFileSize) {
			log.Printf("Buffer should not be %d real: %d\n", len(d.wbuffer.buffer), int(d.extOriginalFileSize))
			d.wbuffer.buffer = append(d.wbuffer.buffer, 0)
		}
		addendumLength := uint32(len(d.extEndFileBuffer))
		for i := uint32(0); i < addendumLength; i++ {
			if d.extEndFileBufferCursor != 0 {
				panic("CIRCULAR BUFFER CURSOR IS NOT ZERO\n")
			}
			d.wbuffer.buffer[d.extOriginalFileSize-addendumLength+i] =
				d.extEndFileBuffer[(d.extEndFileBufferCursor+i)%addendumLength]
		}
	} else {
		d.bitbuffer.flushBits(false)
		bitStreamLen := d.bitbuffer.len()
		lengthHeader := []byte{'a', 'r', 'h', 'c'} // FIXME: deal with lzma headers
		lengthHeader = append(lengthHeader, VERSION_INFORMATION...)
		lengthHeader = append(lengthHeader, d.componentCoalescing)
		originalFileSizeBE := streamLenToBE(d.extOriginalFileSize)
		extensionLength := streamLenToBE(uint32(len(originalFileSizeBE) +
			len(d.extEndFileBuffer)))
		if extensionLength[0] == 0 { // only support 24 bit extension length
			extensionLength = extensionLength[1:]
			lengthHeader = append(lengthHeader, 0x1) // end of buffer extension
			lengthHeader = append(lengthHeader,
				extensionLength...)
			lengthHeader = append(lengthHeader, originalFileSizeBE...)
			lengthHeader = append(lengthHeader, d.extEndFileBuffer[d.extEndFileBufferCursor:]...)
			lengthHeader = append(lengthHeader, d.extEndFileBuffer[:d.extEndFileBufferCursor]...)
		}
		lengthHeader = append(lengthHeader, 0) // end of extensions
		lengthHeader = append(lengthHeader, streamLenToBE(bitStreamLen)...)
		for index := range d.huffMultibuffer {
			d.huffMultibuffer[index].flushBits(false)
			huffMultiStreamLen := d.huffMultibuffer[index].len()
			lengthHeader = append(lengthHeader, streamLenToBE(huffMultiStreamLen)...)
		}
		//fmt.Printf("Length header = %x %d %x\n", lengthHeader, bitStreamLen, bitStreamLen)
		w.Write(lengthHeader)
		d.bitbuffer.flushToWriter(w)
		for _, xhuff := range d.huffMultibuffer {
			xhuff.flushToWriter(w)
		}
	}
	d.wbuffer.flushToWriter(w)
}

// fill fills up the d.bytes.buf buffer from the underlying io.Reader. It
// should only be called when there are no unread bytes in d.bytes.
func (d *decoder) fill() error {
	if d.bytes.i != d.bytes.j {
		panic("jpeg: fill called when unread bytes exist")
	}
	// Move the last 2 bytes to the start of the buffer, in case we need
	// to call unreadByteStuffedByte.
	if d.bytes.j > 2 {
		d.bytes.buf[0] = d.bytes.buf[d.bytes.j-2]
		d.bytes.buf[1] = d.bytes.buf[d.bytes.j-1]
		d.bytes.i, d.bytes.j = 2, 2
	}
	// Fill in the rest of the buffer.
	n, err := d.r.Read(d.bytes.buf[d.bytes.j:])
	if !d.arhc {
		d.extOriginalFileSize += uint32(n)
		for _, byt := range d.bytes.buf[d.bytes.j : d.bytes.j+n] {
			MAX_EXT_END_FILE_BUFFER := 16
			if len(d.extEndFileBuffer) < MAX_EXT_END_FILE_BUFFER || d.extEndEncountered {
				d.extEndFileBuffer = append(d.extEndFileBuffer, byt)
			} else {
				d.extEndFileBuffer[d.extEndFileBufferCursor] = byt
				d.extEndFileBufferCursor += 1
				d.extEndFileBufferCursor %= uint32(len(d.extEndFileBuffer))
			}
		}
	}
	d.bytes.j += n
	if n > 0 {
		err = nil
	}
	return err
}

// unreadByteStuffedByte undoes the most recent readByteStuffedByte call,
// giving a byte of data back from d.bits to d.bytes. The Huffman look-up table
// requires at least 8 bits for look-up, which means that Huffman decoding can
// sometimes overshoot and read one or two too many bytes. Two-byte overshoot
// can happen when expecting to read a 0xff 0x00 byte-stuffed byte.
func (d *decoder) unreadByteStuffedByte() {
	if d.bytes.nUnreadable == 0 {
		panic("jpeg: unreadByteStuffedByte call cannot be fulfilled")
	}
	d.bytes.i -= d.bytes.nUnreadable
	d.bytes.nUnreadable = 0
	d.offset--
	if d.bits.n >= 8 {
		d.bits.a >>= 8
		d.bits.n -= 8
		d.bits.m >>= 8
	}
}

func (d *decoder) appendByteToWriteBuffer(x *byte) {
	if !d.huffTransSect {
		d.wbuffer.appendByte(x)
	}
}

func (d *decoder) appendBytesToWriteBuffer(x []byte) {
	if !d.huffTransSect {
		d.wbuffer.appendBytes(x)
	}
}

// readByte returns the next byte, whether buffered or not buffered. It does
// not care about byte stuffing.
func (d *decoder) readByte() (x byte, err error) {
	for d.bytes.i == d.bytes.j {
		if err = d.fill(); err != nil {
			return 0, err
		}
	}
	defer d.appendByteToWriteBuffer(&x)
	x = d.bytes.buf[d.bytes.i]
	d.bytes.i++
	d.bytes.nUnreadable = 0
	d.offset++
	//fmt.Printf("Read %x\n", x)
	return x, nil
}

// errMissingFF00 means that readByteStuffedByte encountered an 0xff byte (a
// marker byte) that wasn't the expected byte-stuffed sequence 0xff, 0x00.
var errMissingFF00 = FormatError("missing 0xff00 sequence")

// readByteStuffedByte is like readByte but is for byte-stuffed Huffman data.
func (d *decoder) readByteStuffedByte() (x byte, err error) {
	// Take the fast path if d.bytes.buf contains at least two bytes.
	if d.bytes.i+2 <= d.bytes.j {
		x = d.bytes.buf[d.bytes.i]
		d.appendByteToWriteBuffer(&x)
		d.bytes.i++
		d.bytes.nUnreadable = 1
		if x != 0xff {
			return x, err
		}
		if d.bytes.buf[d.bytes.i] != 0x00 {
			return 0, errMissingFF00
		}
		var zero byte
		d.appendByteToWriteBuffer(&zero)
		d.bytes.i++
		d.bytes.nUnreadable = 2
		return 0xff, nil
	}

	x, err = d.readByte()
	if err != nil {
		return 0, err
	}
	if x != 0xff {
		d.bytes.nUnreadable = 1
		return x, nil
	}

	x, err = d.readByte()
	if err != nil {
		d.bytes.nUnreadable = 1
		return 0, err
	}
	d.bytes.nUnreadable = 2
	if x != 0x00 {
		return 0, errMissingFF00
	}
	return 0xff, nil
}

// readFull reads exactly len(p) bytes into p. It does not care about byte
// stuffing.
func (d *decoder) readFull(p []byte) error {
	d.offset += uint32(len(p))
	defer d.appendBytesToWriteBuffer(p)
	// Unread the overshot bytes, if any.
	if d.bytes.nUnreadable != 0 {
		if d.bits.n >= 8 {
			d.unreadByteStuffedByte()
		}
		d.bytes.nUnreadable = 0
	}

	for {
		n := copy(p, d.bytes.buf[d.bytes.i:d.bytes.j])
		p = p[n:]
		d.bytes.i += n
		if len(p) == 0 {
			break
		}
		if err := d.fill(); err != nil {
			if err == io.EOF {
				err = io.ErrUnexpectedEOF
			}
			return err
		}
	}

	return nil
}

// ignore ignores the next n bytes.
func (d *decoder) ignore(n int) error {
	b := make([]byte, n)
	return d.readFull(b)

	// Unread the overshot bytes, if any.
	if d.bytes.nUnreadable != 0 {
		if d.bits.n >= 8 {
			d.unreadByteStuffedByte()
		}
		d.bytes.nUnreadable = 0
	}

	for {
		m := d.bytes.j - d.bytes.i
		if m > n {
			m = n
		}
		d.bytes.i += m
		n -= m
		if n == 0 {
			break
		}
		if err := d.fill(); err != nil {
			if err == io.EOF {
				err = io.ErrUnexpectedEOF
			}
			return err
		}
	}
	return nil
}

func (d *decoder) coalescedComponent(component int) (retval int32) {
	retval = int32(component)
	switch retval {
	case 1:
		if (d.componentCoalescing & comp01coalesce) != 0 {
			retval = 0
		}
	case 2:
		if (d.componentCoalescing & comp12coalesce) != 0 {
			retval = 1
		}
		if (d.componentCoalescing & comp02coalesce) != 0 {
			retval = 0
		}
	}
	return
}

// Specified in section B.2.2.
func (d *decoder) processSOF(n int) error {
	switch n {
	case 6 + 3*nGrayComponent:
		d.nComp = nGrayComponent
	case 6 + 3*nColorComponent:
		d.nComp = nColorComponent
	default:
		return UnsupportedError("SOF has wrong length")
	}
	if err := d.readFull(d.tmp[:n]); err != nil {
		return err
	}
	// We only support 8-bit precision.
	if d.tmp[0] != 8 {
		return UnsupportedError("precision")
	}
	d.height = int(d.tmp[1])<<8 + int(d.tmp[2])
	d.width = int(d.tmp[3])<<8 + int(d.tmp[4])
	if int(d.tmp[5]) != d.nComp {
		return UnsupportedError("SOF has wrong number of image components")
	}
	for i := 0; i < d.nComp; i++ {
		d.comp[i].c = d.tmp[6+3*i]
		d.comp[i].tq = d.tmp[8+3*i]
		if d.nComp == nGrayComponent {
			// If a JPEG image has only one component, section A.2 says "this data
			// is non-interleaved by definition" and section A.2.2 says "[in this
			// case...] the order of data units within a scan shall be left-to-right
			// and top-to-bottom... regardless of the values of H_1 and V_1". Section
			// 4.8.2 also says "[for non-interleaved data], the MCU is defined to be
			// one data unit". Similarly, section A.1.1 explains that it is the ratio
			// of H_i to max_j(H_j) that matters, and similarly for V. For grayscale
			// images, H_1 is the maximum H_j for all components j, so that ratio is
			// always 1. The component's (h, v) is effectively always (1, 1): even if
			// the nominal (h, v) is (2, 1), a 20x5 image is encoded in three 8x8
			// MCUs, not two 16x8 MCUs.
			d.comp[i].h = 1
			d.comp[i].v = 1
			continue
		}
		hv := d.tmp[7+3*i]
		d.comp[i].h = int(hv >> 4)
		d.comp[i].v = int(hv & 0x0f)
		// For color images, we only support 4:4:4, 4:4:0, 4:2:2 or 4:2:0 chroma
		// downsampling ratios. This implies that the (h, v) values for the Y
		// component are either (1, 1), (1, 2), (2, 1) or (2, 2), and the (h, v)
		// values for the Cr and Cb components must be (1, 1).
		if i == 0 {
			if hv != 0x11 && hv != 0x21 && hv != 0x22 && hv != 0x12 {
				return UnsupportedError("luma/chroma downsample ratio")
			}
		} else if hv != 0x11 {
			return UnsupportedError("luma/chroma downsample ratio")
		}
	}
	return nil
}

// Specified in section B.2.4.1.
func (d *decoder) processDQT(n int) error {
	const qtLength = 1 + blockSize
	for ; n >= qtLength; n -= qtLength {
		if err := d.readFull(d.tmp[:qtLength]); err != nil {
			return err
		}
		pq := d.tmp[0] >> 4
		if pq != 0 {
			return UnsupportedError("bad Pq value")
		}
		tq := d.tmp[0] & 0x0f
		if tq > maxTq {
			return FormatError("bad Tq value")
		}
		for i := range d.quant[tq] {
			d.quant[tq][i] = int32(d.tmp[i+1])
		}
	}
	if n != 0 {
		return FormatError("DQT has wrong length")
	}
	return nil
}

// Specified in section B.2.4.4.
func (d *decoder) processDRI(n int) error {
	if n != 2 {
		return FormatError("DRI has wrong length")
	}
	if err := d.readFull(d.tmp[:2]); err != nil {
		return err
	}
	d.ri = int(d.tmp[0])<<8 + int(d.tmp[1])
	return nil
}

// decode reads a JPEG image from r and returns it as an image.Image.
func (d *decoder) decode(r io.Reader, w io.Writer, componentCoalescing uint8) error {
	d.componentCoalescing = componentCoalescing
	d.huffMultibuffer = make([]BitByteStream, 64*3)
	d.r = r
	defer d.flush(w)
	// Check for the ARHC Compressed Image marker.
	// Check for the Start Of Image marker.
	if err := d.readFull(d.tmp[:2]); err != nil {
		return err
	}
	d.arhc = false
	if d.tmp[0] == 'a' && d.tmp[1] == 'r' {
		d.arhc = true
		// read the rest of arhc headers
		if err := d.readFull(d.tmp[:6]); err != nil {
			return err
		}
		if d.tmp[0] == 'h' && d.tmp[1] == 'c' {
		} else {
			return FormatError("arhc header malformed")
		}

		versionExtension := d.tmp[2:6]
		for versionExtension[3] != 0 {
			var sizeBuf [4]byte
			if err := d.readFull(sizeBuf[1:]); err != nil { // only 24 bits of size
				return err
			}
			extension := make([]byte, bufferBEToStreamLength(sizeBuf[:]))
			if err := d.readFull(extension); err != nil { // only 24 bits of size
				return err
			}
			if versionExtension[0] != 1 {
				panic("We only support arhc major version 1 in this decoder")
			}
			d.componentCoalescing = versionExtension[2]
			switch versionExtension[3] {
			case 1:
				// end of file extension
				d.extOriginalFileSize = bufferBEToStreamLength(extension)
				d.extEndFileBuffer = extension[4:]
				d.extEndFileBufferCursor = 0
			default:
				fmt.Printf("Extension unknown 0x%x\n", versionExtension[3])
			}
			if err := d.readFull(versionExtension[3:]); err != nil {
				return err
			}
		}

		if err := d.readFull(d.tmp[6:10]); err != nil {
			return err
		}
		bitVectorLength := bufferBEToStreamLength(d.tmp[6:10])
		if bitVectorLength > 1024*1024*1024 {
			panic(fmt.Sprintf("Bit vector length too large %d", bitVectorLength))
		}
		var huffMultibufferLen [64 * 4 * 3]byte
		d.readFull(huffMultibufferLen[:])
		for index := range d.huffMultibuffer {
			huffMultiStreamLen := bufferBEToStreamLength(huffMultibufferLen[index*4 : index*4+4])
			//fmt.Printf("Making buffer len %d\n", huffMultiStreamLen)
			if huffMultiStreamLen > 256*1024*1024 {
				panic(fmt.Sprintf("Multi bit vector length too large: %d", huffMultiStreamLen))
			}
			d.huffMultibuffer[index].buffer = make([]byte, huffMultiStreamLen)
		}

		bitVector := make([]uint8, bitVectorLength)
		//fmt.Printf("Reading ARHC header of %d bytes\n", len(bitVector))
		d.readFull(bitVector)
		d.bitbuffer.appendBytes(bitVector)

		//fmt.Printf("Reading ARHC header of %d bytes\n", len(huffVector))
		for index := range d.huffMultibuffer {
			d.readFull(d.huffMultibuffer[index].buffer)
		}
		// get jpeg headers
		if err := d.readFull(d.tmp[:2]); err != nil {
			return err
		}
		d.wbuffer.clear()
		d.wbuffer.appendBytes([]byte{0xff, 0xd8})
	}
	if d.tmp[0] != 0xff || d.tmp[1] != soiMarker {
		return FormatError("missing SOI marker")
	}

	// Process the remaining segments until the End Of Image marker.
	for {
		err := d.readFull(d.tmp[:2])
		if err != nil {
			return err
		}
		for d.tmp[0] != 0xff {
			// Strictly speaking, this is a format error. However, libjpeg is
			// liberal in what it accepts. As of version 9, next_marker in
			// jdmarker.c treats this as a warning (JWRN_EXTRANEOUS_DATA) and
			// continues to decode the stream. Even before next_marker sees
			// extraneous data, jpeg_fill_bit_buffer in jdhuff.c reads as many
			// bytes as it can, possibly past the end of a scan's data. It
			// effectively puts back any markers that it overscanned (e.g. an
			// "\xff\xd9" EOI marker), but it does not put back non-marker data,
			// and thus it can silently ignore a small number of extraneous
			// non-marker bytes before next_marker has a chance to see them (and
			// print a warning).
			//
			// We are therefore also liberal in what we accept. Extraneous data
			// is silently ignored.
			//
			// This is similar to, but not exactly the same as, the restart
			// mechanism within a scan (the RST[0-7] markers).
			//
			// Note that extraneous 0xff bytes in e.g. SOS data are escaped as
			// "\xff\x00", and so are detected a little further down below.
			d.tmp[0] = d.tmp[1]
			d.tmp[1], err = d.readByte()
			//fmt.Printf("STRICTLY SPEAKING %x\n", d.tmp[:2])
			if err != nil {
				return err
			}
		}
		marker := d.tmp[1]
		if marker == 0 {
			//fmt.Printf("XTRA DATA %x\n", d.tmp[:2])
			// Treat "\xff\x00" as extraneous data.
			continue
		}
		for marker == 0xff {
			// Section B.1.1.2 says, "Any marker may optionally be preceded by any
			// number of fill bytes, which are bytes assigned code X'FF'".
			marker, err = d.readByte()
			//fmt.Printf("eXTRA DATA %x\n", marker)
			if err != nil {
				return err
			}
		}
		if marker == eoiMarker { // End Of Image.
			//fmt.Printf("EOI %x\n", marker)
			//fmt.Printf("Last bytes of image %x nbits:%d %d\n", d.wbuffer.buffer[len(d.wbuffer.buffer) - 16:], d.wbuffer.nBits, d.wbuffer.bits)
			break
		}
		if rst0Marker <= marker && marker <= rst7Marker {
			//fmt.Printf("Rst %x %x\n", rst0Marker, marker)
			// Figures B.2 and B.16 of the specification suggest that restart markers should
			// only occur between Entropy Coded Segments and not after the final ECS.
			// However, some encoders may generate incorrect JPEGs with a final restart
			// marker. That restart marker will be seen here instead of inside the processSOS
			// method, and is ignored as a harmless error. Restart markers have no extra data,
			// so we check for this before we read the 16-bit length of the segment.
			continue
		}

		// Read the 16-bit length of the segment. The value includes the 2 bytes for the
		// length itself, so we subtract 2 to get the number of remaining bytes.
		if err = d.readFull(d.tmp[:2]); err != nil {
			//fmt.Printf("Seg length %x\n", d.tmp[:2])
			return err
		}
		n := int(d.tmp[0])<<8 + int(d.tmp[1]) - 2
		if n < 0 {
			//fmt.Printf("Short segmnet %d\n", n)
			return FormatError("short segment length")
		}

		switch {
		case marker == sof0Marker || marker == sof2Marker: // Start Of Frame.
			d.progressive = marker == sof2Marker
			err = d.processSOF(n)
		case marker == dhtMarker: // Define Huffman Table.
			err = d.processDHT(n)
		case marker == dqtMarker: // Define Quantization Table.
			err = d.processDQT(n)
		case marker == sosMarker: // Start Of Scan.
			err = d.processSOS(n)
		case marker == driMarker: // Define Restart Interval.
			err = d.processDRI(n)
		case app0Marker <= marker && marker <= app15Marker || marker == comMarker: // APPlication specific, or COMment.
			err = d.ignore(n)
		default:
			//fmt.Printf("UNKNOWN %x\n", marker)
			err = UnsupportedError("unknown marker")
		}
		if err != nil {
			return err
		}
	}
	for {
		_, err := d.readByte()
		if err != nil {
			break
		}
	}

	return nil
}

// Decode reads a JPEG image from r and returns it as an image.Image.
func Decode(r io.Reader, w io.WriteCloser, componentCoalescing uint8) error {
	var d decoder
	err := d.decode(r, w, componentCoalescing)
    err2 := w.Close()
    if err == nil {
        return err2
    }
    return err
}
