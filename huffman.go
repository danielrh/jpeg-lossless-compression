/*  Sirikata Jpeg Texture Transfer Compression -- Texture Transfer management system
 *  huffman.go
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
	"fmt"
	"io"
)

// maxCodeLength is the maximum (inclusive) number of bits in a Huffman code.
const maxCodeLength = 16

// maxNCodes is the maximum (inclusive) number of codes in a Huffman tree.
const maxNCodes = 256

// lutSize is the log-2 size of the Huffman decoder's look-up table.
const lutSize = 8

// huffman is a Huffman decoder, specified in section C.
type huffman struct {
	// length is the number of codes in the tree.
	nCodes int32
	// lut is the look-up table for the next lutSize bits in the bit-stream.
	// The high 8 bits of the uint16 are the encoded value. The low 8 bits
	// are 1 plus the code length, or 0 if the value is too large to fit in
	// lutSize bits.
	lut [1 << lutSize]uint16
	// vals are the decoded values, sorted by their encoding.
	vals [maxNCodes]uint8
	// minCodes[i] is the minimum code of length i, or -1 if there are no
	// codes of that length.
	minCodes [maxCodeLength]int32
	// maxCodes[i] is the maximum code of length i, or -1 if there are no
	// codes of that length.
	maxCodes [maxCodeLength]int32
	// valsIndices[i] is the index into vals of minCodes[i].
	valsIndices [maxCodeLength]int32
	// encodingLut allows easy access to re-encode bits decoded by this huffman
	// As in writer.go's huffmanLUT
	// Each value maps to a uint32 of which the 8 most significant bits hold the
	// codeword size in bits and the 24 least significant bits hold the codeword.
	// The maximum codeword size is 16 bits.
	huffmanLUTencoding [256]uint32
}

// errShortHuffmanData means that an unexpected EOF occurred while decoding
// Huffman data.
var errShortHuffmanData = FormatError("short Huffman data")

// ensureNBits reads bytes from the byte buffer to ensure that d.bits.n is at
// least n. For best performance (avoiding function calls inside hot loops),
// the caller is the one responsible for first checking that d.bits.n < n.
func (d *decoder) ensureNBits(n int32) error {
	for {
		c, err := d.readByteStuffedByte()
		if err != nil {
			if err == io.EOF {
				return errShortHuffmanData
			}
			return err
		}
		d.bits.a = d.bits.a<<8 | uint32(c)
		d.bits.n += 8
		//fmt.Printf("Ensuring %d bits by reading %x: next bits for reading are %d bits %x max:%x\n",
		//    n, c, d.bits.n, d.bits.a, d.bits.m)
		if d.bits.m == 0 {
			d.bits.m = 1 << 7
		} else {
			d.bits.m <<= 8
		}
		if d.bits.n >= n {
			break
		}
	}
	return nil
}

// receiveExtend is the composition of RECEIVE and EXTEND, specified in section
// F.2.2.1.
func (d *decoder) receiveExtend(t uint8) (int32, error) {
	if d.arhc { // here we just return the original value
		val, err := d.bitbuffer.scanBits(uint32(t), false)
		d.wbuffer.emitBits(val, uint32(t), true)
		s := int32(1) << t
		x := int32(val)
		if x < s>>1 {
			x += ((-1) << t) + 1
		}
		return x, err
	}
	if !d.huffTransSect {
		panic("HUFF TRANS SECT SHOULD BE SET")
	}

	if d.bits.n < int32(t) {
		if err := d.ensureNBits(int32(t)); err != nil {
			return 0, err
		}
	}
	d.bits.n -= int32(t)
	d.bits.m >>= t
	s := int32(1) << t
	x := int32(d.bits.a>>uint8(d.bits.n)) & (s - 1)
	if x > 65535 {
		panic("Cnanot decode more than 16 bits at a time")
	}
	//fmt.Printf("Writing raw %x (%d bits)\n", x, t)
	d.bitbuffer.emitBits(uint32(x), uint32(t), false)
	if x < s>>1 {
		x += ((-1) << t) + 1
	}
	return x, nil
}

// processDHT processes a Define Huffman Table marker, and initializes a huffman
// struct from its contents. Specified in section B.2.4.2.
func (d *decoder) processDHT(n int) error {
	for n > 0 {
		if n < 17 {
			return FormatError("DHT has wrong length")
		}
		if err := d.readFull(d.tmp[:17]); err != nil {
			return err
		}
		tc := d.tmp[0] >> 4
		if tc > maxTc {
			return FormatError("bad Tc value")
		}
		th := d.tmp[0] & 0x0f
		if th > maxTh || !d.progressive && th > 1 {
			return FormatError("bad Th value")
		}
		h := &d.huff[tc][th]

		// Read nCodes and h.vals (and derive h.nCodes).
		// nCodes[i] is the number of codes with code length i.
		// h.nCodes is the total number of codes.
		h.nCodes = 0
		var nCodes [maxCodeLength]int32
		for i := range nCodes {
			nCodes[i] = int32(d.tmp[i+1])
			h.nCodes += nCodes[i]
		}
		if h.nCodes == 0 {
			return FormatError("Huffman table has zero length")
		}
		if h.nCodes > maxNCodes {
			return FormatError("Huffman table has excessive length")
		}
		n -= int(h.nCodes) + 17
		if n < 0 {
			return FormatError("DHT has wrong length")
		}
		if err := d.readFull(h.vals[:h.nCodes]); err != nil {
			return err
		}

		// Derive the look-up table.
		for i := range h.lut {
			h.lut[i] = 0
		}
		var x, code uint32
		for i := uint32(0); i < lutSize; i++ {
			code <<= 1
			for j := int32(0); j < nCodes[i]; j++ {
				// The codeLength is 1+i, so shift code by 8-(1+i) to
				// calculate the high bits for every 8-bit sequence
				// whose codeLength's high bits matches code.
				// The high 8 bits of lutValue are the encoded value.
				// The low 8 bits are 1 plus the codeLength.
				base := uint8(code << (7 - i))
				lutValue := uint16(h.vals[x])<<8 | uint16(2+i)
				for k := uint8(0); k < 1<<(7-i); k++ {
					h.lut[base|k] = lutValue
				}
				code++
				x++
			}
		}
		code, x = 0, 0
		for nBits := uint32(0); nBits < maxCodeLength; nBits++ {
			code <<= 1
			for j := int32(0); j < nCodes[nBits]; j++ {
				var encodingValue uint32
				encodingValue = nBits + 1
				encodingValue <<= 24
				encodingValue |= code
				h.huffmanLUTencoding[h.vals[x]] = encodingValue
				code++
				x++
			}
		}

		// Derive minCodes, maxCodes, and valsIndices.
		var c, index int32
		for i, n := range nCodes {
			if n == 0 {
				h.minCodes[i] = -1
				h.maxCodes[i] = -1
				h.valsIndices[i] = -1
			} else {
				h.minCodes[i] = c
				h.maxCodes[i] = c + n - 1
				h.valsIndices[i] = index
				c += n
				index += n
			}
			c <<= 1
		}
	}
	return nil
}

// decodeHuffman returns the next Huffman-coded value from the bit-stream,
// decoded according to h.
func (d *decoder) decodeHuffman(h *huffman, zig int32, component int) (uint8, error) {
	if !d.huffTransSect {
		panic("HUFF TRANS SECT SHOULD BE SET")
	}
	if d.arhc { // here we just return the original value
		unhuffmanValue, err := d.huffMultibuffer[zig+64*d.coalescedComponent(component)].scanBits(8, false)
		// but we also need to write the huffman'd value to the array
		//fmt.Printf("Writing %x => %x to the output\n", unhuffmanValue, h.huffmanLUTencoding[unhuffmanValue])
		hval := h.huffmanLUTencoding[unhuffmanValue]
		d.wbuffer.emitBits(hval&0xffff, hval>>24, true)
		return uint8(unhuffmanValue), err
	}
	if h.nCodes == 0 {
		return 0, FormatError("uninitialized Huffman table")
	}

	if d.bits.n < 8 {
		if err := d.ensureNBits(8); err != nil {
			if err != errMissingFF00 && err != errShortHuffmanData {
				return 0, err
			}
			// There are no more bytes of data in this segment, but we may still
			// be able to read the next symbol out of the previously read bits.
			// First, undo the readByte that the ensureNBits call made.
			//fmt.Printf("Going to unread stuffed byte %d bits %x max:%x\n",
			//    d.bits.n, d.bits.a, d.bits.m)
			d.unreadByteStuffedByte()
			//fmt.Printf("Unread stuffed byte %d bits %x max:%x\n",
			//    d.bits.n, d.bits.a, d.bits.m)

			goto slowPath
		}
	}
	if v := h.lut[(d.bits.a>>uint32(d.bits.n-lutSize))&0xff]; v != 0 {
		n := (v & 0xff) - 1
		reverseHuffmanLookupAssert := uint32(n)
		reverseHuffmanLookupAssert <<= uint32(24)
		reverseHuffmanLookupAssert |= ((d.bits.a >> (uint32(d.bits.n) - uint32(n))) &
			((1 << n) - 1)) // only use that many bits as required from the accumulator
		// make sure our reverse encoding functions properly
		if h.huffmanLUTencoding[uint8(v>>8)] != reverseHuffmanLookupAssert {
			panic(fmt.Sprintf("%d bits read as %x => %x =/=> %x\n",
				n, reverseHuffmanLookupAssert&0xffffff,
				uint8(v>>8), h.huffmanLUTencoding[uint8(v>>8)]))
		}
		//fmt.Printf("%d bits read as %x => %x => %x\n",
		//        n, reverseHuffmanLookupAssert & 0xffffff,
		//        uint8(v >> 8), h.huffmanLUTencoding[uint8(v >> 8)])
		d.bits.n -= int32(n)
		d.bits.m >>= n
		appendedByte := uint8(v >> 8)
		d.huffMultibuffer[zig+64*d.coalescedComponent(component)].appendByte(&appendedByte)
		return uint8(v >> 8), nil
	}

slowPath:
	for i, code := 0, int32(0); i < maxCodeLength; i++ {
		if d.bits.n == 0 {
			if err := d.ensureNBits(1); err != nil {
				return 0, err
			}
		}
		if d.bits.a&d.bits.m != 0 {
			code |= 1
		}
		d.bits.n--
		d.bits.m >>= 1
		if code <= h.maxCodes[i] {
			retval := h.vals[h.valsIndices[i]+code-h.minCodes[i]]
			appendedByte := uint8(retval)
			d.huffMultibuffer[zig+64*d.coalescedComponent(component)].appendByte(&appendedByte)
			//fmt.Printf("%d bits read as %x => %x => %x\n",
			//    i + 1, code, retval, h.huffmanLUTencoding[retval])
			return retval, nil
		}
		code <<= 1
	}
	return 0, FormatError("bad Huffman code")
}

func (d *decoder) decodeBit() (bool, error) {
	bit, err := d.decodeBits(1)
	return bit != 0, err
}

func (d *decoder) decodeBits(n int32) (uint32, error) {
	if d.arhc { // here we just return the original value
		val, err := d.bitbuffer.scanBits(uint32(n), false)
		d.wbuffer.emitBits(val, uint32(n), true)
		return val, err
	}

	if d.bits.n < n {
		if err := d.ensureNBits(n); err != nil {
			return 0, err
		}
	}
	ret := d.bits.a >> uint32(d.bits.n-n)
	ret &= (1 << uint32(n)) - 1
	d.bits.n -= n
	d.bits.m >>= uint32(n)
	if ret > 65535 {
		panic("Cannot decode more than 16 bits at a time")
	}
	//fmt.Printf("Writing xraw %x (%d bits)\n", ret, n);
	d.bitbuffer.emitBits(ret, uint32(n), false)
	return ret, nil
}
