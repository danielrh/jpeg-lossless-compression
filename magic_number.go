/*  Sirikata Jpeg Texture Transfer Compression -- Texture Transfer management system
 *  magic_number.go
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
    "io"
    "errors"
    "fmt"
    "bytes"
)

type MagicNumberInjectionReader struct {
    mBase               io.ReadCloser
    mNewMagic             []byte
    mMagicNumbersReplaced int
}

func NewMagicNumberInjectionReader(r io.ReadCloser, newMagic []byte) MagicNumberInjectionReader{
    var retval MagicNumberInjectionReader
    retval.mBase = r
    retval.mNewMagic = newMagic
    return retval
}

func (mnir *MagicNumberInjectionReader) Close() error {
    return mnir.mBase.Close()
}

func (mnir *MagicNumberInjectionReader) Read(data []byte) (int, error) {
    nRead := 0
    if mnir.mMagicNumbersReplaced < len(mnir.mNewMagic) {
        amountToCopy := len(mnir.mNewMagic) - mnir.mMagicNumbersReplaced
        if len(data) < amountToCopy {
            amountToCopy = len(data)
        }
        copy(data[:amountToCopy],
            mnir.mNewMagic[mnir.mMagicNumbersReplaced:mnir.mMagicNumbersReplaced + amountToCopy])
        mnir.mMagicNumbersReplaced += amountToCopy
        nRead += amountToCopy
    }
    var err error
    if nRead < len(data) {
        var curRead int
        curRead, err = mnir.mBase.Read(data[nRead:])
        nRead += curRead
    }
    return nRead, err
}

type MagicNumberReplacementReader struct {
    mBase                 io.ReadCloser
    mNewMagic             []byte
    mOriginalMagic        []byte
    mMagicNumbersReplaced int
}

func NewMagicNumberReplacementReader(r io.ReadCloser,
    originalMagic []byte, newMagic []byte) (mrr MagicNumberReplacementReader) {

    mrr.mOriginalMagic = originalMagic;
    mrr.mNewMagic = newMagic;
    mrr.mBase = r;
    mrr.mMagicNumbersReplaced = 0;
    if len(mrr.mOriginalMagic) != len(mrr.mNewMagic) {
        panic("Magic numbers must be the same length");
    }
    return
}
func (mrr *MagicNumberReplacementReader) Close() error {
    return mrr.mBase.Close()
}
func (mrr *MagicNumberReplacementReader) Read(data []byte) (bytesRead int, err error) {
    bytesRead, err = mrr.mBase.Read(data);
    for off := 0;
        mrr.mMagicNumbersReplaced < len(mrr.mOriginalMagic) && off < bytesRead;
        off++ {

        if data[off] != mrr.mOriginalMagic[mrr.mMagicNumbersReplaced] {
            err = errors.New(fmt.Sprintf("Magic Number Mismatch: %x != %x",
                data[:len(mrr.mOriginalMagic) - mrr.mMagicNumbersReplaced],
                mrr.mOriginalMagic[mrr.mMagicNumbersReplaced:]))
        }
        data[off] = mrr.mNewMagic[mrr.mMagicNumbersReplaced]
        mrr.mMagicNumbersReplaced++
    }
    return
}

type MagicNumberReplacementWriter struct {
    mBase                 io.WriteCloser
    mNewMagic             []byte
    mOriginalMagic        []byte
    mMagicNumbersReplaced int
}

func NewMagicNumberReplacementWriter(w io.WriteCloser,
    originalMagic []byte,
    newMagic []byte) (mrw MagicNumberReplacementWriter) {

    mrw.mOriginalMagic = originalMagic;
    mrw.mNewMagic = newMagic;
    mrw.mBase = w;
    mrw.mMagicNumbersReplaced = 0;
    if len(mrw.mOriginalMagic) != len(mrw.mNewMagic) {
       panic("Magic numbers must be the same length")
    }
    return
}
func (mrw* MagicNumberReplacementWriter) Write(data []byte) (int, error) {
    if mrw.mMagicNumbersReplaced < len(mrw.mOriginalMagic) {
        if len(data) > len(mrw.mOriginalMagic) - mrw.mMagicNumbersReplaced {
            if !bytes.Equal(data[:len(mrw.mOriginalMagic) - mrw.mMagicNumbersReplaced],
                mrw.mOriginalMagic[mrw.mMagicNumbersReplaced:]) {
                return 0, errors.New(fmt.Sprintf("Trying to write mismatching bytes %x %x",
                                     data[:len(mrw.mOriginalMagic) - mrw.mMagicNumbersReplaced],
                                      mrw.mOriginalMagic[mrw.mMagicNumbersReplaced:]))
            }
            replacedMagic := make([]byte, len(data))
            copy(replacedMagic, data)
            for index, magic := range mrw.mNewMagic[mrw.mMagicNumbersReplaced:] {
                replacedMagic[index - mrw.mMagicNumbersReplaced] = magic
            }
            mrw.mMagicNumbersReplaced = len(mrw.mOriginalMagic)
            return mrw.mBase.Write(replacedMagic);
        } else {
            if !bytes.Equal(data, mrw.mOriginalMagic[mrw.mMagicNumbersReplaced:mrw.mMagicNumbersReplaced + len(data)]) {
                return 0, errors.New(fmt.Sprintf("Trying to write mismatching bytes %x %x",
                          data, mrw.mOriginalMagic[mrw.mMagicNumbersReplaced:]))
            }
            mrw.mMagicNumbersReplaced += len(data);
            return mrw.mBase.Write(mrw.mNewMagic[mrw.mMagicNumbersReplaced :
                mrw.mMagicNumbersReplaced + len(data)]);
        }
    } else {
        return mrw.mBase.Write(data);
    }
}
func (mrw *MagicNumberReplacementWriter) Close() error {
    return mrw.mBase.Close();
}

