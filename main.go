/*  Sirikata Jpeg Texture Transfer Compression -- Texture Transfer management system
 *  main.go
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

import "flag"
import "os"
import "fmt"
import "io"

//import "github.com/danielrh/go-xz"

func main() {
	fp := os.Stdin
	of := os.Stdout
	inputFileName := flag.String("input", "", "input file to operate on")
	outputFileName := flag.String("output", "", "output file to print to")
	shouldDecompress := flag.Bool("decompress", false, "True if file should be decompressed")
	xz := flag.Bool("xz", false, "Avoid using ARHC for compression")
	coalesceYCb := flag.Bool("coalesceYCb", false, "Coalesce Y and Cb")
	coalesceYCr := flag.Bool("coalesceYCr", false, "Coalesce Y and Cr")
	coalesceCbCr := flag.Bool("coalesceCbCr", false, "Coalesce Cb and Cr")
	flag.Parse()
	var err error
	if len(*inputFileName) != 0 {
		fp, err = os.Open(*inputFileName)
		if err != nil {
			panic(err)
		}
	}
	if len(*outputFileName) != 0 {
		of, err = os.Create(*outputFileName)
		if err != nil {
			panic(err)
		}
	}
	componentCoalescing := uint8(0)
	if *coalesceYCb {
		componentCoalescing |= comp01coalesce
	}
	if *coalesceYCr {
		componentCoalescing |= comp02coalesce
	}
	if *coalesceCbCr {
		componentCoalescing |= comp12coalesce
	}
	var magic [6]byte
	nMagic, err := io.ReadFull(fp, magic[:])
	isArhc := DecodeIsARHC(magic[:nMagic])
	isXz := DecodeIs7z(magic[:nMagic])
	isJpeg := DecodeIsJPEG(magic[:nMagic])
	inputReader := NewMagicNumberInjectionReader(fp, magic[:nMagic])
	if *shouldDecompress {
		if isXz {
			err = Decompress7ZtoAny(&inputReader, of)
		} else if isArhc {
			err = DecompressARHCtoJPEG(&inputReader, of)
		} else {
			panic(fmt.Sprintf("Magic Number %x not known", magic))
		}
	} else {
		if isJpeg && !*xz {
			err = CompressJPEGtoARHC(&inputReader, of, componentCoalescing)
		} else {
			err = CompressAnyto7Z(&inputReader, of)
		}
	}
	/* COMPRESSION EXAMPLE
	   cw := NewCompressionWriter(of);
	   for {
	       var buffer [4096]byte
	       nRead, readErr := fp.Read(buffer[:])
	       if readErr != nil && readErr != io.EOF {
	           panic(readErr)
	       }
	       nWrite, writeErr := cw.Write(buffer[:nRead])
	       if writeErr != nil {
	           panic(writeErr)
	       }
	       _ = nWrite
	       if readErr != nil {
	           break
	       }
	   }
	   fp.Close()
	   cw.Close()

	   dr := xz.NewDecompressionReader(fp);
	   for {
	       var buffer [4096]byte
	       nRead, readErr := dr.Read(buffer[:])
	       if readErr != nil && readErr != io.EOF {
	           panic(readErr)
	       }
	       nWrite, writeErr := of.Write(buffer[:nRead])
	       if writeErr != nil || nWrite < nRead{
	           panic(writeErr)
	       }
	       if readErr != nil {
	           break
	       }
	   }
	   dr.Close()
	   of.Close()
	*/

	if err != nil {
		panic(err)
	}
	//     err = Encode(of, img, nil)
	//     if err != nil {
	//        panic(err)
	//     }
	fp.Close()
	of.Close()
}
