/*  Sirikata Jpeg Texture Transfer Compression -- Texture Transfer management system
 *  arhc.go
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
import "bytes"
import "io"
import "github.com/danielrh/go-xz"
var MAGIC_7Z = []byte { 0xFD, '7', 'z', 'X', 'Z', 0x00 };
var MAGIC_ARHC = []byte { 'A', 'R', 'H', 'C', 0x01, 0x00 };


func DecodeIs7z(data []byte) bool {
    minSize := len(data)
    if len(data) >  len(MAGIC_7Z) {
        minSize = len(MAGIC_7Z)
    }
    return bytes.Equal(data[:minSize], MAGIC_7Z[:minSize])
}
func DecodeIsJPEG(data []byte) bool {
    if len(data) == 1 {
       return data[0] == 0xff
    }
    return len(data) >= 2 && data[0] == 0xff && data[1] == 0xd8;
}
func DecodeIsARHC(data []byte) bool {
    minSize := len(data)
    if len(data) > len(MAGIC_ARHC) {
        minSize = len(MAGIC_ARHC)
    }
    return bytes.Equal(data[:minSize], MAGIC_ARHC[:minSize]);
}


// Decode reads a JPEG image from r and returns it as an image.Image.
func CompressJPEGtoARHC(r io.Reader, w io.Writer, componentCoalescing uint8) error {
    magic := NewMagicNumberReplacementWriter(&xz.NopCloseWriteWrapper{w}, MAGIC_7Z, MAGIC_ARHC);
    cw := xz.NewCompressionWriter(&magic);
    return Decode(r, &cw, componentCoalescing);
}

// Decode reads a JPEG image from r and returns it as an image.Image.
func DecompressARHCtoJPEG(r io.Reader, w io.WriteCloser) error {
    magic := NewMagicNumberReplacementReader(&xz.NopCloseReadWrapper{r}, MAGIC_ARHC, MAGIC_7Z);
    cr := xz.NewDecompressionReader(&magic);
    return Decode(&cr, w, 0);
}

func Copy(r io.ReadCloser, w io.WriteCloser) error {
    var buffer [16384]byte;
    for {
        nRead, readErr := r.Read(buffer[:]);
        if (nRead == 0) {
            w.Close();
            return nil;
        }
        offset := 0;
        nWritten, writeErr := w.Write(buffer[offset:nRead])
        offset += nWritten
        if writeErr != nil {
            w.Close();
            return writeErr
        }
        if readErr != nil {
            w.Close();
            if (readErr == io.EOF) {
                return nil
            }
            return readErr;
        }
    }
}

// Decode reads a JPEG image from r and returns it as an image.Image.
func CompressAnyto7Z(r io.Reader, w io.Writer) error {
    cw := xz.NewCompressionWriter(&xz.NopCloseWriteWrapper{w});
    return Copy(&xz.NopCloseReadWrapper{r}, &cw);
}
// Decode reads a JPEG image from r and returns it as an image.Image.
func Decompress7ZtoAny(r io.Reader, w io.Writer) error {
    cr := xz.NewDecompressionReader(&xz.NopCloseReadWrapper{r});
    return Copy(&cr, &xz.NopCloseWriteWrapper{w});
}
