# GIFPlayer
GIF player with four buttons for RP2040 and Waveshare LCD display.

This program is intended for RP2040 especially with Waveshare LCD display.
It shows four gif's selectable by user with four buttons. Gif's are saved
to local FAT filesystem via USB.

Gif's are named after which button activates it. Format is Xbtn.gif where
X is a button number. Gif's must be same size than display. In this case
240x240 pixels.

Program uses DMA and full frame buffer to update display.

Links to source codes that was helping during development process
https://learn.adafruit.com/mini-gif-players/coding-the-mini-gif-players
https://github.com/MarkTillotson/PicoSPI
https://github.com/stienman/Raspberry-Pi-Pico-PIO-LCD-Driver

---------------------------------------------------------------------------
Parts of this program is lisenced under following licenses

Copyright 2022 Limor Fried for Adafruit Industries
License MIT

(c) 2023 Dmitry Grinberg  https://dmitry.gr
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
