<h1>ACARS demodulator plugin</h1>

<h2>Introduction</h2>

This plugin can be used to demodulate ACARS, VDL-2, HFDL and Aero data packets, broadcast by aircraft and satellites.
These packets can contain flight plans as well as status information about the aircraft.

ACARS uses MSK modulation at 2400 baud, which is then AM modulated on to the VHF carrier.
In Europe, the main ACARS frequencies are:

  - 131.525 MHz (SITA - Europe secondary) 
  - 131.725 MHz (SITA - Europe primary)
  - 131.825 MHz (ARINC/Collins)
 
VDL Mode 2 uses D8PSK modulation at 10500 baud (bit rate 31500 bps). It can contain ACARS or ATN packets.
VDL Mode 2 frequencies in Europe are:

  - 136.675 MHz (ARINC)
  - 136.725 MHz (ARINC)
  - 136.775 MHz (SITA)
  - 136.800 MHz
  - 136.825 MHz (ARINC)
  - 136.875 MHz (SITA)
  - 136.900 MHz
  - 136.975 MHz (SITA & ARINC, the common signalling channel)

HFDL uses M-PSK at 1800 baud and frequencies are between 2 and 21 MHz. The active frequencies vary with propagation, and the 
frequencies in use are reported in squitters transmitted by the groundstations. These are combined by the demodulator and
displayed in the Ground Station Table dialog. You can find an initial frequency to try on: https://hfdl.observer/

Inmarsat Classic Aero (aviation SATCOM). The ground station to aircraft
(P channel) downlink is in L band around:

| Region              | Id    | Satellite				  | Longitude | G->A Frequency   |
|---------------------|-------|---------------------------|-----------|------------------|
| Atlantic Ocean West | AOR-W | Inmarsat 4-F3             | 98W       |                  |
| Atlantic Ocean East | AOR-E | Inmarsat 3-F5             | 54W       | ~1,545 MHz       |
| Pacific Ocean       | POR   | Inmarsat 4-F1             | 178E      |                  |
| Indian Ocean        | IOR   | Inmarsat 4A-F4 / Alphasat | 25E       |                  |

<h2>Interface</h2>

![ACARS Demodulator plugin GUI](../../../doc/img/AcarsDemod_plugin.png)

<h3>1: Frequency shift from center frequency of reception</h3>

Use the wheels to adjust the frequency shift in Hz from the center frequency of reception. Left click on a digit sets the cursor position at this digit. Right click on a digit sets all digits on the right to zero. This effectively floors value at the digit position. Wheels are moved with the mousewheel while pointing at the wheel or by selecting the wheel with the left mouse click and using the keyboard arrows. Pressing shift simultaneously moves digit by 5 and pressing control moves it by 2.

<h3>2: Channel power</h3>

Average total power in dB relative to a +/- 1.0 amplitude signal received in the pass band.

<h3>3: Level meter in dB</h3>

  - top bar (green): average value
  - bottom bar (blue green): instantaneous peak value
  - tip vertical bar (bright green): peak hold value

<h3>4: Mode and Aero rate</h3>

Selects the demodulator: ACARS, VDL-2, HFDL or Aero - and, in Aero mode
only, the rate and channel selector appears beside it. 

Changing either resets the demodulator and sets a default RF bandwidth to suit.

  - ACARS: plain old ACARS, 2400 baud MSK amplitude modulated onto VHF
  - VDL-2: VDL Mode 2, D8PSK at 10500 baud
  - Aero: Inmarsat Classic Aero, at 600, 1200 or 10500 bps
  - HFDL: HF Data Link, 1800 baud PSK at 300 to 1800 bps (Bitrate is automatically detected)

Each demodulator runs at its own fixed channel sample rate, which the channelizer is
set to when the mode is selected.

<table>
  <tr><th>Mode</th><th>Channel sample rate</th><th>Symbol rate</th><th>Samples per symbol</th></tr>
  <tr><td>ACARS</td><td>48 kS/s</td><td>2400 Bd</td><td>20</td></tr>
  <tr><td>VDL-2</td><td>105 kS/s</td><td>10500 Bd</td><td>10</td></tr>
  <tr><td>HFDL</td><td>7.2 kS/s</td><td>1800 Bd</td><td>4</td></tr>
  <tr><td>Aero, 600 bps</td><td>19.2 kS/s</td><td>600 Bd</td><td>32</td></tr>
  <tr><td>Aero, 1200 bps</td><td>19.2 kS/s</td><td>1200 Bd</td><td>16</td></tr>
  <tr><td>Aero, 10500 bps</td><td>105 kS/s</td><td>5250 Bd</td><td>20</td></tr>
</table>

<h3>5: RF Bandwidth</h3>

This specifies the bandwidth of a LPF that is applied to the input signal to limit the RF bandwidth. 

<h3>6: Threshold</h3>

ARACS pre-key (preamble) detection threshold, as a fraction from 0 to 1. The statistic it is compared against
is the depth of 2400 Hz modulation on the envelope. A real pre-key
reads about 0.85 and noise reads near zero, so the default of 0.25 sits between them and is
a false alarm setting rather than something to tune per station. Lower it only if
transmissions are being missed entirely.

<h3>7: Filter field</h3>

Selects which column of the messages table the filter applies to: Registration, Flight, Label or Message.

<h3>8: Filter pattern</h3>

Entering a regular expression displays only messages where the selected column matches it. Clear the field to show all messages.

<h3>9: UDP</h3>

When checked, received packets are forwarded over UDP. Right click the button for the
destination address and port; the button's tooltip shows where they are being sent.

<h3>10: Hide frames with no information</h3>

Some frames carry nothing a listener can use - no aircraft is named and no message is
carried - and on a busy channel they are the majority of what is received. This button
hides them, and is checked by default. Uncheck it to see everything that has been
received.

In **Aero**, these are the satellite housekeeping the ground earth station broadcasts
continuously, whether or not any aircraft is logged on: the system tables that describe
the satellite, its beams and its channels, the data EIRP table, the fill-in signal units
that pad a frame out to length, and the reserved types nobody has documented. On a
P channel these are most of what is transmitted.

In **VDL-2**, these are the supervisory and link management responses - `RR`, `RNR`,
`REJ`, `SREJ`, `DM`, `DISC`, `UA`, `FRMR` and `U`.

<h3>11: Display Chart</h3>

When checked, displays a chart plotting the number of frames received per second.

<h3>12: Display Ground Station Table</h3>

When checked, displays a table of the ground stations heard in squitters and the frequencies each announced as in use.
Right clicking a row offers finding that station on the Map, or retuning the device so this channel receives that frequency.

<h3>13: Feed</h3>

When checked, forwards received ACARS messages to community aggregators, as acarsdec-style JSON - one object per UDP datagram, or a newline-delimited stream over TCP. 
Right click the button for settings: a station identifier sent with every message, and per-service enables with host, port and protocol.

<h3>14: Download OpenSky Database</h3>

<h3>15: Start/stop Logging ACARS packets to .csv File</h3>

When checked, writes all received ACARS packets to a .csv file. The file has two columns,
`DateTime` and `Data`: the date and time as ISO 8601 with milliseconds, and the packet as
hexadecimal. Earlier versions wrote separate `Date` and `Time` columns in the local
format; those files are no longer read.

<h3>16: .csv Log Filename</h3>

Click to specify the name of the .csv file which received ACARS packets are logged to.

<h3>17: Read Data from .csv File</h3>

Click to specify a previously written ACARS .csv log file, which is read and used to update the ACARS messages table.

<h3>18: Clear Messages from table</h3>

Pressing this button clears all messages from the table.

<h3>Frames per Second Chart</h3>

The chart button toggles a chart plotting the number of frames received per second, as one
series per protocol:

  - **ACARS** - ACARS messages, from whichever protocol is selected. A message carried over
    VDL-2, HFDL or Aero counts here, not under its protocol, because it is the same kind of
    message however it arrived.
  - **VDL-2 link** - AVLC link frames that are not ACARS: XID and GSIF link management, and
    supervisory acknowledgements whether or not they are displayed in the table.
  - **HFDL** - squitters and logons.
  - **Aero** - log-ons, log-offs, channel assignments and acknowledgements.

Clicking a legend entry shows or hides that series; right clicking the
chart button clears the data.
The mouse wheel zooms the time axis, or the rate axis with shift held, centred on the
cursor; dragging with the left button pans. The time axis follows the newest data whenever
its right edge is at the latest point. Data more than 10 minutes old is averaged over one
minute intervals, so long sessions do not accumulate an excessive number of points.

<h3>Received Messages Table</h3>

The received messages table displays the contents of the ACARS packets that have been received.

<h3>Message Decode</h3>

The decode field displays a decode of the ACARS message selected in the received messages table.
For multipart messages it shows the assembly status, the combined message - decoded once all
parts have arrived - and the selected block.

* Date/Time - The receiver's local date and time when the ACARS message was received, or the recorded date and time when reading a CSV log. Right click the table to choose between showing the date and time or the time alone; the column sorts by the full date and time either way.
* Dir - The transmission direction. An up arrow is an uplink from a ground station to an aircraft; a down arrow is a downlink from an aircraft to a ground station.
* GS - Ground Station Id. ACARS: ARINC 618 Mode character. VDL-2: Ground Station (GS). HFDL: Ground Station (GS). Aero: Ground Earth Station (GES).
* GS Decode - Ground Station name. Right clicking either ground station column offers *Find on map*, which draws that station on the Map as a fixed antenna and centres on it. Only HFDL and VDL-2 stations have a known position - HFDL's comes from the system table and VDL-2's from the community ground station list - so the action is absent for an ACARS Mode character, which names no station, and for an Aero GES id, which resolves only to a satellite and an ocean region.
* Registration - The aircraft address or registration from the seven-character ACARS header field, with leading padding periods removed.
* Ack - The technical acknowledgement field. It is displayed as `ACK`, `NAK`, or the block ID to which the acknowledgement applies.
* Label - The two-character message label identifying the message type. When present, a sublabel is appended after a slash, for example `H1/CF`.
* Label Decode - A human-readable description of the message label or sublabel.
* ID - The block identifier used for message sequencing and acknowledgement. Downlinks normally use `0`-`9`, while uplinks normally use `A`-`Z`.
* Origin - For downlinks, the one-character originator code identifying the aircraft system or terminal that generated the message.
* Origin Decode - The decoded name of the downlink originator, such as FMC, CMU, TCAS, or a cabin terminal.
* Msg No. - For downlinks, the two-digit message number in the message text header.
* Block Seq. - For multipart downlinks, the block sequence character used to order the blocks.
* Flight - For downlinks, the six-character flight identifier carried in the message text header.
* Message - The ACARS message payload after the standard downlink header fields and any recognized sublabel have been removed.
* Message Decode - A structured, human-readable decode of the message payload, flattened to a single
  semicolon-separated line so more fits in the column. Selecting the row displays the full multi-line
  decode below the table, with field labels bolded; the same multi-line form is what the Aircraft
  feature shows in its Map popups.
  Beyond the ARINC 622/623 ATS messages (CPDLC, oceanic and departure clearances, ATIS), decoders are included for
  position reports (labels 10, 12, 16, 22, 24, 44 and H1 FMS reports, which also feed the Lat/Lon columns),
  airline operational reports (label 80 POSRPT/OPNORM/INRANG), OOOI event reports (labels 13-18 and QP-QT),
  flight status reports (label 15 FST01), H1 FMS flight plans and route inserts,
  weather requests (label 5U) and frequency autotune (label :;). Several of these are airline formats without public
  documentation; the field layouts follow [airframes.io's community documentation](https://github.com/airframesio/acars-message-documentation) and observed traffic.
* ATC - Air Traffic Control communication extracted from supported ARINC 622/CPDLC messages.
* Lat (°) - Aircraft latitude extracted from an ADS-C or CPDLC position report or any decoded message carrying a position (position report labels, OOOI events, FST01 flight status, H1 FMS reports), in decimal degrees; north is positive and south is negative.
* Lon (°) - Aircraft longitude extracted from the same sources, in decimal degrees; east is positive and west is negative.
* Alt (ft) - Aircraft altitude extracted from an ADS-C or CPDLC position report, in feet.
* Hex - The complete received ACARS frame rendered as hexadecimal bytes, including its framing, header, payload, check sequence, and terminating byte.
* Protocol - Which link the frame arrived on: `ACARS`, `VDL-2`, `HFDL`, or `Aero` followed by the channel it came on - `Aero P`, `Aero R` or `Aero T` - since on Aero the channel is what says which link it was. This is kept separate from Label Decode, which describes what the Label means and nothing else.
* Rate (bps) - The bit rate the frame was sent at: 2400 for ACARS, 31500 for VDL-2, 300 to 1800 for HFDL and 600, 1200 or 10500 for Aero. Note that VDL-2's 31500 bps is carried by 10500 symbols per second of D8PSK, three bits to a symbol, so this figure and the symbol rate quoted under Mode (4) differ for that protocol alone.

Both columns are added at the right hand end rather than beside Dir where they belong, because saved column widths and positions are stored by column number and inserting in the middle would shift every one of them. Drag them wherever suits; the position is remembered.

<h3>VDL-2 Link Frames</h3>

In VDL-2 mode, ACARS messages carried over AVLC fill the table exactly as described above.
AVLC frames that do not carry an ACARS message (ATN data, XID/GSIF link management, and -
unless hidden by (10) - supervisory acknowledgements) have no ACARS header, so only the
following columns are used:

* Date/Time - The receiver's local date and time when the frame was received.
* Dir - The transmission direction. An up arrow is an uplink to an aircraft; a down arrow is a downlink from an aircraft.
* GS - Ground Station Id. 
* GS Decode - Ground Station name.
* Registration - The aircraft side of the link. VDL-2 link frames identify aircraft by their 24-bit ICAO address (the same address the aircraft transmits in Mode S / ADS-B) rather than by registration, so the registration is looked up from the OpenSky database (14); when the address is not in the database, the address itself is shown in hexadecimal. Frames between ground stations show `GS` followed by the ground station's address, and broadcasts show `All`.
* Label - The AVLC frame type: `I` (information), `UI`, `XID` (link management, including GSIF ground station broadcasts), `TEST`, or the supervisory types `RR`, `RNR`, `REJ`, `SREJ`, `DM`, `DISC`, `UA`, `FRMR`.
* Label Decode - What the frame type means, for example `Information`, `Exchange identification (link management)`, `Receive ready` or `Selective reject - retransmit this frame`.
* Message - The source and destination addresses of the frame, followed for information frames by the decoded protocol stack, layer by layer, for example `X.25 DATA > CLNP > COTP DT > CPDLC`, and for XID frames by the message type, for example `XID: Handoff Initiation` or `XID: Ground Station Information Frame`. X.25 call setup, clear and supervisory packets, ES-IS and IDRP routing traffic are identified by name.
* Message Decode - For information frames carrying an ICAO ATN application, the full decode of the CM (context management logon), CPDLC or ADS-C version 2 message. For XID frames, the decoded public and VDL-specific link parameters: connection management, XID sequencing, frequency support, ground station location, destination airport, ATN router NETs and so on. Multi-line decodes are flattened with semicolons; hover over the cell (or select the row) for the original multi-line form.
* ATC - For frames carrying a CPDLC controller-pilot message, the expanded message elements and their arguments, for example `WILCO` or `REQUEST [level], Flight level: 370` - the ATN equivalent of the DO-219 expansion shown for ACARS ATC messages. Left blank for CPDLC frames with no message data.
* Lat (°) / Lon (°) - The position from an XID aircraft or ground station location parameter, or from an ADS-C version 2 report, in decimal degrees. The XID encoding is coarse (0.1 degrees, roughly 10 km); ADS-C positions are precise.
* Alt (ft) - The altitude from an XID aircraft location parameter (1000 ft resolution) or an ADS-C version 2 report, in feet.
* Hex - The complete AVLC frame rendered as hexadecimal bytes: address fields, link control, information field and frame check sequence.

<h3>HFDL Frames</h3>

HFDL frames that are not carrying an ACARS message use the same columns as the VDL-2 link
frames below. The Label is the LPDU type as its hexadecimal octet and the Label Decode is
what that type is - `Logon request`, `Unnumbered data` and so on. A squitter is not an
LPDU and has no type octet, so it is labelled `Squitter`, decoded as
`Ground station frequency broadcast`.

<h3>Aero Signal Units</h3>

In Aero mode, ACARS messages carried over the satellite link are reassembled from their
signal units inside the demodulator and fill the table exactly as described above. Signal
units that are not carrying an ACARS message use only the following columns:

* Date/Time - The receiver's local date and time when the signal unit was received.
* Dir - The transmission direction. An up arrow is a transmission from the ground earth
  station on a P channel; a down arrow is a transmission from an aircraft on an R or T
  channel.
* GS - Ground Earth Station (GES) Id. 
* GS Decode - Satellite name, for example 'AOR-E'. See frequency table above.
* Registration - The aircraft, when the signal unit names one. The AES ID is the
  aircraft's 24-bit ICAO address - the same address it transmits in Mode S / ADS-B - so
  the registration is looked up from the OpenSky database (14); when the address is not
  in the database, the address itself is shown in hexadecimal. Blank for signal units
  that name no aircraft.
* Label - The signal unit type as its hexadecimal octet, for example `10`, `32` or `0C`.
* Label Decode - What that type is, for example `Log-on request`, `C channel assignment,
  flight safety` or `System table, satellite identification`.
* Message - The two ends of the exchange, as `GES 2A` and the aircraft, in the direction
  of transmission. Signal units carrying neither an AES ID nor a GES ID show
  `Broadcast` instead: the system and EIRP tables describe the satellite and its beams
  rather than addressing any one aircraft, so there is no conversation to show. Most
  `Broadcast` rows are hidden by default; uncheck (10) to see them.
* Message Decode - The decoded contents, where the unit has any: the pair of L band
  frequencies and beam type in a voice channel assignment, the log-on parameters, and so
  on.
* Hex - The complete signal unit as hexadecimal bytes, with the type in the first octet.

<h2>Attribution</h2>

Aircraft images are from [PlaneSpotters](https://www.planespotters.net/)

The GUI uses [libacars](https://github.com/szpajder/libacars) by Tomasz Lemiech.

The VDL Mode 2 physical layer constants were cross-checked against, and the ICAO ATN
ASN.1 decoder in the `atn` directory (used for CM, CPDLC and ADS-C version 2 carried over
X.25/CLNP/COTP) is vendored from [dumpvdl2](https://github.com/szpajder/dumpvdl2) by
Tomasz Lemiech (GPL-3.0).

The HFDL constants and conventions were cross-checked against
[dumphfdl](https://github.com/szpajder/dumphfdl) by Tomasz Lemiech (GPL-3.0).

The Inmarsat Classic Aero protocol parameters - the convolutional code and its polynomials,
the interleaver row permutation, the scrambler polynomial and seed, the unique words, the
frame and signal unit layouts, and the ISU/SSU reassembly rules - were all established by
reading [JAERO](https://github.com/jontio/JAERO) by Jonti Olds (MIT).

The community VDL Mode 2 ground station list is vendored from [acars-vdl2 @ Groups.io](https://acars-vdl2.groups.io/).

Some code was written using AI.
