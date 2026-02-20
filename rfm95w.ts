/**
 * LoRa radio driver for the HopeRF RFM95W / Semtech SX127x chipset.
 * Lets micro:bit programs send and receive text and raw bytes over long-range
 * LoRa radio without an internet connection.
 */
//% color=#e67e22 icon="\uf1eb" weight=8
namespace Lora {

    /**
     * Frequency band to use.  Choose the band that matches the radio module
     * you purchased and the regulations in your country.
     */
    //% block="Lora"
    export enum Band {
        //% block="EU (868 MHz)"
        EU868 = 868,
        //% block="AU / US (915 MHz)"
        AU915 = 915
    }

    /**
     * LoRa spreading factor.  Higher values give longer range at the cost of
     * slower data rate.  Both ends of a link must use the same spreading factor.
     */
    export enum SpreadingFactor {
        //% block="SF7 (fastest, shortest range)"
        SF7 = 7,
        //% block="SF8"
        SF8 = 8,
        //% block="SF9"
        SF9 = 9,
        //% block="SF10"
        SF10 = 10,
        //% block="SF11"
        SF11 = 11,
        //% block="SF12 (slowest, longest range)"
        SF12 = 12
    }

    /**
     * LoRa signal bandwidth.  Narrower bandwidth gives longer range;
     * wider bandwidth gives faster data rate.  Both ends must match.
     */
    export enum Bandwidth {
        //% block="7.8 kHz"
        BW7k8 = 0,
        //% block="10.4 kHz"
        BW10k4 = 1,
        //% block="15.6 kHz"
        BW15k6 = 2,
        //% block="20.8 kHz"
        BW20k8 = 3,
        //% block="31.25 kHz"
        BW31k25 = 4,
        //% block="41.7 kHz"
        BW41k7 = 5,
        //% block="62.5 kHz"
        BW62k5 = 6,
        //% block="125 kHz (default)"
        BW125k = 7,
        //% block="250 kHz"
        BW250k = 8,
        //% block="500 kHz (fastest, shortest range)"
        BW500k = 9
    }

    /**
     * LoRa forward error-correction coding rate.  Higher rates add more
     * redundancy so packets survive noisier conditions, but reduce throughput.
     * Both ends must match.
     */
    export enum CodingRate {
        //% block="4/5 (default)"
        CR45 = 1,
        //% block="4/6"
        CR46 = 2,
        //% block="4/7"
        CR47 = 3,
        //% block="4/8 (most error correction)"
        CR48 = 4
    }

    // --- SX127x register map (subset) ---
    const REG_FIFO = 0x00
    const REG_OP_MODE = 0x01
    const REG_FRF_MSB = 0x06
    const REG_FRF_MID = 0x07
    const REG_FRF_LSB = 0x08
    const REG_PA_CONFIG = 0x09
    const REG_LNA = 0x0C
    const REG_FIFO_ADDR_PTR = 0x0D
    const REG_FIFO_TX_BASE_ADDR = 0x0E
    const REG_FIFO_RX_BASE_ADDR = 0x0F
    const REG_FIFO_RX_CURRENT_ADDR = 0x10
    const REG_IRQ_FLAGS = 0x12
    const REG_RX_NB_BYTES = 0x13
    const REG_PKT_SNR_VALUE = 0x19
    const REG_PKT_RSSI_VALUE = 0x1A
    const REG_RSSI_VALUE = 0x1B
    const REG_MODEM_CONFIG_1 = 0x1D
    const REG_MODEM_CONFIG_2 = 0x1E
    const REG_PREAMBLE_MSB = 0x20
    const REG_PREAMBLE_LSB = 0x21
    const REG_PAYLOAD_LENGTH = 0x22
    const REG_MODEM_CONFIG_3 = 0x26
    const REG_SYNC_WORD = 0x39
    const REG_DIO_MAPPING_1 = 0x40
    const REG_VERSION = 0x42
    const REG_PA_DAC = 0x4D

    // --- Modes / bits ---
    const MODE_LONG_RANGE = 0x80 // LoRa mode bit in RegOpMode
    const MODE_SLEEP = 0x00
    const MODE_STDBY = 0x01
    const MODE_TX = 0x03
    const MODE_RX_CONTINUOUS = 0x05

    // IRQ flags
    const IRQ_RX_DONE = 0x40
    const IRQ_TX_DONE = 0x08
    const IRQ_PAYLOAD_CRC_ERROR = 0x20
    const IRQ_CLEAR_ALL = 0xFF

    // BW lookup table: bandwidth in kHz × 100 (integer arithmetic, no floats)
    const BW_KHZ_X100 = [780, 1040, 1560, 2080, 3125, 4170, 6250, 12500, 25000, 50000]

    // Wiring defaults (override if you want later)
    let _nss = DigitalPin.P16
    let _rst = DigitalPin.P8
    let _dio0 = DigitalPin.P1

    let _inited = false
    let _rxHandler: (receivedString: string) => void = null
    let _lastRssi = 0
    let _lastSnrRaw = 0   // raw REG_PKT_SNR_VALUE (uint8 representing signed byte)
    let _sf = 7           // current spreading factor (7–12)
    let _bwCode = 7       // current BW register code (default 125 kHz)

    function csSelect() { pins.digitalWritePin(_nss, 0) }
    function csDeselect() { pins.digitalWritePin(_nss, 1) }

    function spiInit() {
        // micro:bit default SPI pins are P13/P14/P15
        pins.spiPins(DigitalPin.P15, DigitalPin.P14, DigitalPin.P13)
        pins.spiFrequency(1000000) // start conservative; can go higher later
        pins.spiFormat(8, 0) // 8 bits, mode 0
        pins.digitalWritePin(_nss, 1)
    }

    function readReg(addr: number): number {
        csSelect()
        pins.spiWrite(addr & 0x7F)
        const v = pins.spiWrite(0x00)
        csDeselect()
        return v
    }

    function writeReg(addr: number, value: number): void {
        csSelect()
        pins.spiWrite(addr | 0x80)
        pins.spiWrite(value)
        csDeselect()
    }

    function burstWrite(addr: number, buf: Buffer): void {
        csSelect()
        pins.spiWrite(addr | 0x80)
        for (let i = 0; i < buf.length; i++) pins.spiWrite(buf[i])
        csDeselect()
    }

    function burstRead(addr: number, len: number): Buffer {
        const out = pins.createBuffer(len)
        csSelect()
        pins.spiWrite(addr & 0x7F)
        for (let i = 0; i < len; i++) out[i] = pins.spiWrite(0x00)
        csDeselect()
        return out
    }

    function resetChip() {
        pins.digitalWritePin(_rst, 0)
        basic.pause(2)
        pins.digitalWritePin(_rst, 1)
        basic.pause(10)
    }

    function setMode(mode: number) {
        writeReg(REG_OP_MODE, MODE_LONG_RANGE | mode)
    }

    function setFrequencyMHz(mhz: number) {
        // FRF = (freq_hz * 2^19) / 32e6
        const freqHz = mhz * 1000000
        const frf = Math.floor(freqHz * 524288 / 32000000)
        writeReg(REG_FRF_MSB, (frf >> 16) & 0xFF)
        writeReg(REG_FRF_MID, (frf >> 8) & 0xFF)
        writeReg(REG_FRF_LSB, frf & 0xFF)
    }

    function applyLdro() {
        const needLdro = ((1 << _sf) * 100) > (16 * BW_KHZ_X100[_bwCode])
        let v = readReg(REG_MODEM_CONFIG_3)
        writeReg(REG_MODEM_CONFIG_3, needLdro ? (v | 0x08) : (v & 0xF7))
    }

    function configRadioDefaults(band: Band) {
        // FIFO bases
        writeReg(REG_FIFO_TX_BASE_ADDR, 0x00)
        writeReg(REG_FIFO_RX_BASE_ADDR, 0x00)
        writeReg(REG_FIFO_ADDR_PTR, 0x00)

        // Preamble
        writeReg(REG_PREAMBLE_MSB, 0x00)
        writeReg(REG_PREAMBLE_LSB, 0x08) // 8 symbols

        // Sync word: 0x34 is common for "public LoRa" style networks;
        // for your own kid-network you can pick something else (0x12 etc.)
        writeReg(REG_SYNC_WORD, 0x34)

        // Frequency
        if (band == Band.EU868) setFrequencyMHz(868.0)
        else setFrequencyMHz(915.0)

        // Modem config:
        // BW=125kHz, CR=4/5, Explicit header
        // RegModemConfig1: BW bits 7..4, CR bits 3..1
        writeReg(REG_MODEM_CONFIG_1, 0x72) // 0b0111_0010
        // RegModemConfig2: SF=7 (0x70), CRC on (bit2)
        writeReg(REG_MODEM_CONFIG_2, 0x74) // SF7 + CRC on
        // RegModemConfig3: LowDataRateOptimize off, AGC auto on
        writeReg(REG_MODEM_CONFIG_3, 0x04)

        // LNA boost
        writeReg(REG_LNA, readReg(REG_LNA) | 0x03)

        // PA config: PA_BOOST, modest power
        // 0x80 selects PA_BOOST. Lower 4 bits are output power.
        writeReg(REG_PA_CONFIG, 0x80 | 0x0F)
        writeReg(REG_PA_DAC, 0x84)   // default; 0x87 enables 20 dBm

        // Map DIO0: in LoRa mode, DIO0 can be RxDone (00) or TxDone (01) depending on mode.
        // We'll just leave mapping at default 0 and read IRQ flags by polling.
        writeReg(REG_DIO_MAPPING_1, 0x00)
    }

    // Event IDs for internal use. App code can use onReceivedString/onSent which are mapped to these.
    const LORA_EVENT_ID = 3110
    const INT_DIO0 = 0
    const APP_TX = 2

    function setDio0Mapping(mapping: number) {
        // mapping: 0b00 => RxDone, 0b01 => TxDone (LoRa mode)
        let v = readReg(REG_DIO_MAPPING_1)
        v = (v & 0x3F) | ((mapping & 0x03) << 6)
        writeReg(REG_DIO_MAPPING_1, v)
    }

    function setupDio0Interrupt() {
        pins.setPull(_dio0, PinPullMode.PullDown)

        // DIO0 goes high on mapped events; treat it like a pulse/edge.
        pins.onPulsed(_dio0, PulseValue.High, function () {
            // Keep ISR work tiny
            control.raiseEvent(LORA_EVENT_ID, INT_DIO0)
        })

        control.onEvent(LORA_EVENT_ID, INT_DIO0, function () {
            const flags = readReg(REG_IRQ_FLAGS)

            // RxDone path
            if ((flags & IRQ_RX_DONE) != 0) {
                if ((flags & IRQ_PAYLOAD_CRC_ERROR) != 0) {
                    writeReg(REG_IRQ_FLAGS, IRQ_CLEAR_ALL)
                    return
                }

                // Datasheet-correct FIFO read sequence
                const currentAddr = readReg(REG_FIFO_RX_CURRENT_ADDR)
                writeReg(REG_FIFO_ADDR_PTR, currentAddr)
                const len = readReg(REG_RX_NB_BYTES)
                const pkt = burstRead(REG_FIFO, len)

                _lastRssi = readReg(REG_PKT_RSSI_VALUE) - 157
                _lastSnrRaw = readReg(REG_PKT_SNR_VALUE)

                writeReg(REG_IRQ_FLAGS, IRQ_CLEAR_ALL)

                const msg = pkt.toString()
                if (_rxHandler != null) {
                    control.runInBackground(function () { _rxHandler(msg) })
                }
            }

            // TxDone path
            if ((flags & IRQ_TX_DONE) != 0) {
                writeReg(REG_IRQ_FLAGS, IRQ_CLEAR_ALL)
                setDio0Mapping(0)
                setMode(MODE_RX_CONTINUOUS)
                control.raiseEvent(LORA_EVENT_ID, APP_TX)
            }
        })
    }

    /**
     * Initialise the RFM95W radio module and start listening for incoming
     * packets.  Call this once at the start of your program before using any
     * other LoRa blocks.
     * @param band frequency band matching your module and local regulations, eg: Lora.Band.EU868
     */
    //% block="LoRa init band %band"
    //% group="Configuration"  weight=100
    export function init(band: Band) {
        spiInit()
        resetChip()

        const version = readReg(REG_VERSION)
        // SX1276/77/78/79 typically reports 0x12.
        if (version != 0x12) {
            _inited = false
            return
        }

        setMode(MODE_SLEEP)
        basic.pause(10)
        setMode(MODE_STDBY)
        basic.pause(10)

        configRadioDefaults(band)

        // Clear IRQs and go to RX continuous by default
        writeReg(REG_IRQ_FLAGS, IRQ_CLEAR_ALL)
        setMode(MODE_RX_CONTINUOUS)

        setupDio0Interrupt()

        // Start in RX, so map DIO0 -> RxDone
        setDio0Mapping(0) // 00
        writeReg(REG_IRQ_FLAGS, IRQ_CLEAR_ALL)
        setMode(MODE_RX_CONTINUOUS)

        _inited = true
    }

    /**
     * Send a text string over LoRa.  The string is encoded as UTF-8 and
     * transmitted immediately.  The radio returns to receive mode automatically
     * once transmission is complete.
     * @param value the string to send, eg: "Hello!"
     */
    //% block="LoRa send string %value"
    //% value.shadowOptions.toString=true
    //% group="Send"  weight=60
    export function sendString(value: string): void {
        if (!_inited) return
        const buf = control.createBufferFromUTF8(value)
        sendBuffer(buf)
    }

    /**
     * Send a raw buffer of bytes over LoRa.  Use this when you need to send
     * binary data or a custom packet format.  Maximum recommended payload is
     * 240 bytes at SF7/BW125.
     * @param buf the buffer to transmit
     */
    //% group="Send"  weight=59  advanced=true
    export function sendBuffer(buf: Buffer): void {
        if (!_inited) return

        setMode(MODE_STDBY)
        writeReg(REG_IRQ_FLAGS, IRQ_CLEAR_ALL)

        // DIO0 -> TxDone while transmitting
        setDio0Mapping(1) // 01

        writeReg(REG_FIFO_ADDR_PTR, 0x00)
        burstWrite(REG_FIFO, buf)
        writeReg(REG_PAYLOAD_LENGTH, buf.length)

        setMode(MODE_TX)

        // No polling loop needed anymore if you use onSent().
        // But: keep a fallback timeout if you want robustness.
    }

    /**
     * Run code when a LoRa string is received.  The ``receivedString``
     * reporter contains the text that arrived.
     */
    //% block="on LoRa received %receivedString"
    //% draggableParameters="reporter"
    //% group="Receive"  weight=50
    export function onReceivedString(handler: (receivedString: string) => void): void {
        _rxHandler = handler
    }

    /**
     * Run code when a transmission finishes and the radio has returned to
     * receive mode.  Useful for sequencing back-to-back transmissions or
     * showing a "sent" indicator without blocking.
     */
    //% block="on LoRa message sent"
    //% group="Send"  advanced=true  weight=40
    export function onSent(handler: () => void) {
        control.onEvent(LORA_EVENT_ID, APP_TX, handler)
    }

    /**
     * RSSI of the last received packet in dBm.  Typical values range from
     * about −70 dBm (strong, nearby) to −110 dBm (weak, at the edge of range).
     * Returns 0 if no packet has been received yet.
     */
    //% block="LoRa signal strength (dBm)"
    //% group="Receive"  weight=30
    export function receivedSignalStrength(): number {
        return _lastRssi
    }

    /**
     * Signal-to-noise ratio of the last received packet in dB.  Positive
     * values indicate a clean signal; negative values mean the packet arrived
     * below the noise floor (still decodable by LoRa, but marginal).
     * Returns 0 if no packet has been received yet.
     */
    //% block="LoRa received SNR (dB)"
    //% group="Receive"  weight=29
    export function receivedSnr(): number {
        const raw = _lastSnrRaw > 127 ? _lastSnrRaw - 256 : _lastSnrRaw
        return raw / 4
    }

    /**
     * Wideband RSSI of the radio channel right now, in dBm.  Unlike
     * ``receivedSignalStrength``, this does not require a packet to have
     * arrived — it reflects the background noise level on the channel and can
     * be used to check whether the channel is busy before transmitting.
     * Returns 0 if the radio has not been initialised.
     */
    //% block="LoRa channel RSSI (dBm)"
    //% group="Receive"  weight=28
    export function channelRssi(): number {
        if (!_inited) return 0
        return readReg(REG_RSSI_VALUE) - 157
    }

    /**
     * Set the transmit output power.  Higher power reaches further but draws
     * more current from the battery.  Both ends do not need to match.
     * @param dBm transmit power in dBm, between 2 and 20, eg: 17
     */
    //% block="LoRa set TX power %dBm dBm"
    //% dBm.min=2 dBm.max=20
    //% group="Configuration"  advanced=true  weight=90
    export function setTxPower(dBm: number): void {
        if (!_inited) return
        if (dBm == 20) {
            writeReg(REG_PA_CONFIG, 0x80 | 0x0F)
            writeReg(REG_PA_DAC, 0x87)
        } else {
            const power = Math.max(0, Math.min(15, dBm - 2))
            writeReg(REG_PA_CONFIG, 0x80 | power)
            writeReg(REG_PA_DAC, 0x84)
        }
    }

    /**
     * Set the LoRa spreading factor.  Higher spreading factors give longer
     * range at the cost of slower transmission.  Both the sender and receiver
     * must use the same spreading factor or they will not be able to
     * communicate.  Default is SF7.
     * @param sf the spreading factor to use, eg: Lora.SpreadingFactor.SF7
     */
    //% block="LoRa set spreading factor %sf"
    //% group="Configuration"  advanced=true  weight=80
    export function setSpreadingFactor(sf: SpreadingFactor): void {
        if (!_inited) return
        _sf = sf
        const current = readReg(REG_MODEM_CONFIG_2)
        writeReg(REG_MODEM_CONFIG_2, (current & 0x0F) | (sf << 4))
        applyLdro()
    }

    /**
     * Set the LoRa signal bandwidth.  Narrower bandwidths give better
     * sensitivity and range; wider bandwidths give faster data rates.  Both
     * the sender and receiver must use the same bandwidth.  Default is 125 kHz.
     * @param bw the bandwidth to use, eg: Lora.Bandwidth.BW125k
     */
    //% block="LoRa set bandwidth %bw"
    //% group="Configuration"  advanced=true  weight=70
    export function setBandwidth(bw: Bandwidth): void {
        if (!_inited) return
        _bwCode = bw
        const current = readReg(REG_MODEM_CONFIG_1)
        writeReg(REG_MODEM_CONFIG_1, (current & 0x0F) | (bw << 4))
        applyLdro()
    }

    /**
     * Set the forward error-correction coding rate.  Higher coding rates add
     * more redundancy so packets survive noisier conditions, at the cost of
     * slightly lower throughput.  Both the sender and receiver must match.
     * Default is 4/5.
     * @param cr the coding rate to use, eg: Lora.CodingRate.CR45
     */
    //% block="LoRa set coding rate %cr"
    //% group="Configuration"  advanced=true  weight=60
    export function setCodingRate(cr: CodingRate): void {
        if (!_inited) return
        const current = readReg(REG_MODEM_CONFIG_1)
        writeReg(REG_MODEM_CONFIG_1, (current & 0xF1) | (cr << 1))
    }

    /**
     * Set the LoRa sync word.  Only radios sharing the same sync word will
     * receive each other's packets, so this can be used to create isolated
     * networks.  ``0x34`` is the public LoRa default.  Both ends must match.
     * @param word sync word byte (0–255), eg: 0x34
     */
    //% block="LoRa set sync word %word"
    //% word.min=0 word.max=255
    //% group="Configuration"  advanced=true  weight=50
    export function setSyncWord(word: number): void {
        if (!_inited) return
        writeReg(REG_SYNC_WORD, word)
    }
}
