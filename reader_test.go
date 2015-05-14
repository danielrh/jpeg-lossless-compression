/*  Sirikata Jpeg Texture Transfer Test Suite -- Texture Transfer management system
 *  reader_test.go
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

// Copyright 2012 The Go Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package main

import (
	"bytes"
	"testing"
)

func assertEqual(t *testing.T, a uint32, b int) {
	if int(a) != b {
		t.Errorf("%x != %x\n", a, b)
	}
}

func TestStuffedZeroBitStream(t *testing.T) {
	var b BitByteStream
	b.appendBytes([]byte{0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00})
	ff, _ := b.scanBits(8, true)
	assertEqual(t, ff, 0xff)
	ff, _ = b.scanBits(16, true)
	assertEqual(t, ff, 0xffff)
	ff, _ = b.scanBits(1, true)
	assertEqual(t, ff, 0x1)
	ff, _ = b.scanBits(16, true)
	assertEqual(t, ff, 0xffff)
	sf, _ := b.scanBits(7, true)
	assertEqual(t, sf, 0x7f)
	var sb BitByteStream
	sb.appendBytes([]byte{0x81, 0xff, 0x00, 0x81, 0x80})
	one, _ := sb.scanBits(1, true)
	assertEqual(t, one, 0x1)
	zero, _ := sb.scanBits(6, true)
	assertEqual(t, zero, 0)
	oneff, _ := sb.scanBits(10, true)
	assertEqual(t, oneff, 0x3ff)
	zero, _ = sb.scanBits(6, true)
	assertEqual(t, zero, 0)
	three, _ := sb.scanBits(2, true)
	assertEqual(t, three, 3)
	zero, _ = sb.scanBits(6, true)
	assertEqual(t, zero, 0)
	one, _ = sb.scanBits(1, true)
	assertEqual(t, one, 0x0)
	var b2 BitByteStream
	b2.emitBits(0xff, 8, true)
	b2.emitBits(0xffff, 16, true)
	b2.emitBits(0x1, 1, true)
	b2.emitBits(0xffff, 16, true)
	b2.emitBits(0x7f, 7, true)
	b2.flushBits(true)
	if !bytes.Equal(b.buffer, b2.buffer) {
		t.Errorf("%x != %x\n", b.buffer, b2.buffer)
	}
	var sb2 BitByteStream
	sb2.emitBits(0x1, 1, true)
	sb2.emitBits(0x0, 6, true)
	sb2.emitBits(0x3ff, 10, true)
	sb2.emitBits(0x0, 6, true)
	sb2.emitBits(0x3, 2, true)
	sb2.emitBits(0x0, 6, true)
	sb2.emitBits(0x0, 1, true)
	sb2.flushBits(true)
	if !bytes.Equal(sb.buffer, sb2.buffer) {
		t.Errorf("%x != %x\n", sb.buffer, sb2.buffer)
	}

}

func TestRawBitStream(t *testing.T) {
	var b BitByteStream
	b.appendBytes([]byte{0xff, 0xff, 0xff, 0xff, 0xff, 0xff})
	ff, _ := b.scanBits(8, false)
	assertEqual(t, ff, 0xff)
	ff, _ = b.scanBits(16, false)
	assertEqual(t, ff, 0xffff)
	ff, _ = b.scanBits(1, false)
	assertEqual(t, ff, 0x1)
	ff, _ = b.scanBits(16, false)
	assertEqual(t, ff, 0xffff)
	sf, _ := b.scanBits(7, false)
	assertEqual(t, sf, 0x7f)
	var sb BitByteStream
	sb.appendBytes([]byte{0x81, 0xff, 0x81, 0x80})
	one, _ := sb.scanBits(1, false)
	assertEqual(t, one, 0x1)
	zero, _ := sb.scanBits(6, false)
	assertEqual(t, zero, 0)
	oneff, _ := sb.scanBits(10, false)
	assertEqual(t, oneff, 0x3ff)
	zero, _ = sb.scanBits(6, false)
	assertEqual(t, zero, 0)
	three, _ := sb.scanBits(2, false)
	assertEqual(t, three, 3)
	zero, _ = sb.scanBits(6, false)
	assertEqual(t, zero, 0)
	one, _ = sb.scanBits(1, false)
	assertEqual(t, one, 0x0)
	var b2 BitByteStream
	b2.emitBits(0xff, 8, false)
	b2.emitBits(0xffff, 16, false)
	b2.emitBits(0x1, 1, false)
	b2.emitBits(0xffff, 16, false)
	b2.emitBits(0x7f, 7, false)
	b2.flushBits(false)
	if !bytes.Equal(b.buffer, b2.buffer) {
		t.Errorf("%x != %x\n", b.buffer, b2.buffer)
	}
	var sb2 BitByteStream
	sb2.emitBits(0x1, 1, false)
	sb2.emitBits(0x0, 6, false)
	sb2.emitBits(0x3ff, 10, false)
	sb2.emitBits(0x0, 6, false)
	sb2.emitBits(0x3, 2, false)
	sb2.emitBits(0x0, 6, false)
	sb2.emitBits(0x0, 1, false)
	sb2.flushBits(false)
	if !bytes.Equal(sb.buffer, sb2.buffer) {
		t.Errorf("%x != %x\n", sb.buffer, sb2.buffer)
	}
	var asymm BitByteStream
	asymm.appendBytes([]byte{0x00, 0xc0})
	o256, _ := asymm.scanBits(9, false)
	assertEqual(t, o256, 0x1)
	x40, _ := asymm.scanBits(7, false)
	assertEqual(t, x40, 0x40)
	var asymm2 BitByteStream
	asymm2.emitBits(0x1, 9, false)
	asymm2.emitBits(0x40, 7, false)
	if !bytes.Equal(asymm.buffer, asymm2.buffer) {
		t.Errorf("%x != %x\n", asymm.buffer, asymm2.buffer)
	}

}
