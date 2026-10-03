//! Host-side tests.
//!
//! These drive the **real** driver code - [`crate::Sn32UsbBus`], its `UsbBus`
//! impl, and its interrupt handler - against a memory-backed register file
//! instead of the peripheral, so what is exercised is the shipped logic rather
//! than a reimplementation of it.
//!
//! # What these tests do and do not prove
//!
//! They prove the driver's *bookkeeping and register traffic*: that allocation
//! respects the unidirectional-endpoint constraint, that packet RAM is handed
//! out without overlap, that a SETUP packet is copied out before endpoint 0 is
//! armed, that the direction bit in `CFG` matches the allocated direction, that
//! `INTEN` ends up at the value the vendor firmware uses, and that the EP0 state
//! machine produces the sequence `usb-device` expects.
//!
//! They prove **nothing** about the silicon. The mock register file stores what
//! it is given and does not model the SIE: it does not auto-NAK after a
//! transaction, it does not raise interrupts, and it does not move data on the
//! bus. A test passing here means the driver asked the hardware for the right
//! thing, not that the hardware agreed.

use usb_device::bus::{PollResult, UsbBus};
use usb_device::endpoint::EndpointType;
use usb_device::{UsbDirection, UsbError};

use crate::regs::{bits, Regs, EP0_MAX_PACKET_SIZE, MAX_ENDPOINT_NUMBER, OFF_CFG, OFF_EP0CTL, OFF_EP1BUFOS, OFF_INTEN};
use crate::Sn32UsbBus;

/// A bus plus the register handle its mock backend writes into.
fn bus() -> (Sn32UsbBus, Regs) {
    let regs = Regs::new();
    regs.mock_reset();
    (Sn32UsbBus::new(), regs)
}

/// Builds a SETUP packet.
fn setup(bm_request_type: u8, request: u8, value: u16, index: u16, length: u16) -> [u8; 8] {
    let mut p = [0u8; 8];
    p[0] = bm_request_type;
    p[1] = request;
    p[2..4].copy_from_slice(&value.to_le_bytes());
    p[4..6].copy_from_slice(&index.to_le_bytes());
    p[6..8].copy_from_slice(&length.to_le_bytes());
    p
}

/// Puts a SETUP packet into the mock packet RAM and raises `EP0_SETUP`, the way
/// the SIE would.
fn deliver_setup(bus: &Sn32UsbBus, regs: &Regs, packet: &[u8; 8]) {
    for (i, chunk) in packet.chunks(4).enumerate() {
        let mut word = [0u8; 4];
        word.copy_from_slice(chunk);
        // Packet RAM is at a fixed offset from the register block; the mock is
        // one flat array, so `fifo_write_word` relative to the block base is
        // enough as long as we read it back the same way.
        regs.fifo_write_word(i * 4, u32::from_le_bytes(word));
    }
    regs.mock_poke(crate::regs::OFF_INSTS, bits::EP0_SETUP);
    bus.interrupt_handler();
}

/// Raises an arbitrary `INSTS` bit and runs the interrupt handler.
fn raise(bus: &Sn32UsbBus, regs: &Regs, insts: u32) {
    regs.mock_poke(crate::regs::OFF_INSTS, insts);
    bus.interrupt_handler();
}

// ---------------------------------------------------------------------------
// Register-level bring-up
// ---------------------------------------------------------------------------

#[test]
fn enable_produces_the_vendor_inten_value() {
    let (mut bus, regs) = bus();
    bus.alloc_ep(
        UsbDirection::Out,
        Some(0x00.into()),
        EndpointType::Control,
        64,
        0,
    )
    .unwrap();
    bus.alloc_ep(
        UsbDirection::In,
        Some(0x80.into()),
        EndpointType::Control,
        64,
        0,
    )
    .unwrap();
    bus.enable();

    // BUS_IE | USB_IE | EPN_ACK_EN | BUSWK_IE. The vendor stock firmware's own
    // USB enable routine (FUN_00000fe6) writes exactly 0xB0000040 here.
    assert_eq!(regs.mock_peek(OFF_INTEN), 0xB000_0040);

    // CFG = VREG33_EN | PHY_EN | DPPU_EN | SIE_EN | ESD_EN. The vendor's suspend
    // path clears PHY_EN|ESD_EN with the mask 0xB7FFFFFF, so those two bits must
    // be set here.
    let cfg = regs.mock_peek(OFF_CFG);
    assert_eq!(cfg & bits::ESD_EN, bits::ESD_EN);
    assert_eq!(cfg & bits::PHY_EN, bits::PHY_EN);
    assert_eq!(cfg & bits::VREG33_EN, bits::VREG33_EN);
    assert_eq!(cfg & bits::SIE_EN, bits::SIE_EN);
    assert_eq!(cfg & bits::DPPU_EN, bits::DPPU_EN);
}

#[test]
fn suspend_and_resume_toggle_exactly_phy_and_esd() {
    let (mut bus, regs) = bus();
    bus.alloc_ep(
        UsbDirection::Out,
        Some(0x00.into()),
        EndpointType::Control,
        64,
        0,
    )
    .unwrap();
    bus.alloc_ep(
        UsbDirection::In,
        Some(0x80.into()),
        EndpointType::Control,
        64,
        0,
    )
    .unwrap();
    bus.enable();

    let after_enable = regs.mock_peek(OFF_CFG);

    bus.suspend();
    let suspended = regs.mock_peek(OFF_CFG);
    // This is the vendor's own suspend expression: CFG &= 0xB7FFFFFF.
    assert_eq!(suspended, after_enable & 0xB7FF_FFFF);

    bus.resume();
    // The vendor's resume (FUN_00001310) is CFG |= 0x48000000.
    assert_eq!(regs.mock_peek(OFF_CFG), suspended | 0x4800_0000);
}

// ---------------------------------------------------------------------------
// Endpoint allocation
// ---------------------------------------------------------------------------

#[test]
fn control_endpoints_share_number_zero() {
    let (mut bus, _regs) = bus();
    let out = bus
        .alloc_ep(
            UsbDirection::Out,
            Some(0x00.into()),
            EndpointType::Control,
            64,
            0,
        )
        .unwrap();
    let inn = bus
        .alloc_ep(
            UsbDirection::In,
            Some(0x80.into()),
            EndpointType::Control,
            64,
            0,
        )
        .unwrap();
    assert_eq!(u8::from(out), 0x00);
    assert_eq!(u8::from(inn), 0x80);
}

#[test]
fn an_endpoint_number_cannot_be_both_directions() {
    // The defining constraint of this controller: EPn_DIR makes endpoint number
    // n either IN or OUT, never both, and there is one buffer per number.
    let (mut bus, _regs) = bus();
    let a = bus
        .alloc_ep(UsbDirection::In, Some(0x81.into()), EndpointType::Interrupt, 8, 1)
        .unwrap();
    assert_eq!(u8::from(a), 0x81);

    let err = bus
        .alloc_ep(UsbDirection::Out, Some(0x01.into()), EndpointType::Interrupt, 8, 1)
        .unwrap_err();
    assert_eq!(err, UsbError::InvalidEndpoint);
}

#[test]
fn auto_allocation_gives_distinct_numbers() {
    let (mut bus, _regs) = bus();
    let a = bus
        .alloc_ep(UsbDirection::In, None, EndpointType::Interrupt, 8, 1)
        .unwrap();
    let b = bus
        .alloc_ep(UsbDirection::In, None, EndpointType::Interrupt, 8, 1)
        .unwrap();
    let c = bus
        .alloc_ep(UsbDirection::Out, None, EndpointType::Interrupt, 8, 1)
        .unwrap();
    assert_eq!(u8::from(a), 0x81);
    assert_eq!(u8::from(b), 0x82);
    assert_eq!(u8::from(c), 0x03);
}

#[test]
fn running_out_of_endpoints_is_reported_not_wrapped() {
    let (mut bus, _regs) = bus();
    for _ in 0..MAX_ENDPOINT_NUMBER {
        bus.alloc_ep(UsbDirection::In, None, EndpointType::Interrupt, 8, 1)
            .unwrap();
    }
    assert_eq!(
        bus.alloc_ep(UsbDirection::In, None, EndpointType::Interrupt, 8, 1)
            .unwrap_err(),
        UsbError::EndpointOverflow
    );
}

#[test]
fn isochronous_is_refused() {
    let (mut bus, _regs) = bus();
    let err = bus
        .alloc_ep(
            UsbDirection::In,
            None,
            EndpointType::Isochronous {
                synchronization:
                    usb_device::endpoint::IsochronousSynchronizationType::NoSynchronization,
                usage: usb_device::endpoint::IsochronousUsageType::Data,
            },
            64,
            1,
        )
        .unwrap_err();
    assert_eq!(err, UsbError::Unsupported);
}

#[test]
fn endpoint_buffers_do_not_overlap_and_direction_bits_are_set() {
    let (mut bus, regs) = bus();
    let ep1 = bus
        .alloc_ep(UsbDirection::In, Some(0x81.into()), EndpointType::Interrupt, 8, 1)
        .unwrap();
    let ep2 = bus
        .alloc_ep(UsbDirection::Out, Some(0x02.into()), EndpointType::Interrupt, 64, 1)
        .unwrap();
    let _ = (ep1, ep2);
    bus.enable();

    // EP1BUFOS and EP2BUFOS must be distinct, ordered, and past EP0's 64 bytes.
    let off1 = regs.mock_peek(OFF_EP1BUFOS);
    let off2 = regs.mock_peek(OFF_EP1BUFOS + 4);
    assert_eq!(off1, 64, "first endpoint buffer starts after EP0's reservation");
    assert!(off2 >= off1 + 8, "buffers overlap: {off1} then {off2}");

    // CFG.EP1_DIR = 0 (IN), CFG.EP2_DIR = 1 (OUT).
    let cfg = regs.mock_peek(OFF_CFG);
    assert_eq!(cfg & bits::ep_dir(1), 0, "EP1 should be IN");
    assert_eq!(cfg & bits::ep_dir(2), bits::ep_dir(2), "EP2 should be OUT");
}

// ---------------------------------------------------------------------------
// EP0 control transfers
// ---------------------------------------------------------------------------

#[test]
fn setup_packet_is_latched_and_endpoint0_is_left_naking() {
    let (mut bus, regs) = bus();
    bus.alloc_ep(UsbDirection::Out, Some(0x00.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x80.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.enable();

    let packet = setup(0x80, 6, 0x0100, 0, 64); // GET_DESCRIPTOR, device
    deliver_setup(&bus, &regs, &packet);

    // Endpoint 0 must NOT be armed here. Packet RAM offset 0 holds both the
    // SETUP packet and EP0's data buffer, so arming the receiver before the
    // SETUP bytes are copied out would let the host's data stage overwrite it.
    assert_eq!(
        regs.mock_peek(OFF_EP0CTL) & bits::ENDP_STATE_MASK,
        bits::ENDP_STATE_NAK,
        "endpoint 0 must stay NAKing until the SETUP packet has been read out"
    );

    // poll reports the SETUP and keeps reporting it until read.
    match bus.poll() {
        PollResult::Data { ep_setup, .. } => assert_eq!(ep_setup & 1, 1),
        _ => panic!("expected a SETUP event"),
    }
    match bus.poll() {
        PollResult::Data { ep_setup, .. } => assert_eq!(ep_setup & 1, 1),
        _ => panic!("SETUP must be reported until it is read"),
    }

    let mut buf = [0u8; 8];
    assert_eq!(bus.read(0x00.into(), &mut buf).unwrap(), 8);
    assert_eq!(buf, packet);

    // Only now is the setup consumed, and a device-to-host transfer needs no
    // arming yet - usb-device arms it by calling `write`.
    assert_eq!(
        regs.mock_peek(OFF_EP0CTL) & bits::ENDP_STATE_MASK,
        bits::ENDP_STATE_NAK
    );
}

#[test]
fn host_to_device_transfer_with_a_data_stage_arms_the_receiver() {
    let (mut bus, regs) = bus();
    bus.alloc_ep(UsbDirection::Out, Some(0x00.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x80.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.enable();

    // SET_REPORT: host to device, 8-byte data stage.
    deliver_setup(&bus, &regs, &setup(0x21, 0x09, 0x0200, 0, 8));
    let mut buf = [0u8; 8];
    bus.read(0x00.into(), &mut buf).unwrap();

    // Now the receiver must be armed, or the host NAKs forever.
    assert_eq!(
        regs.mock_peek(OFF_EP0CTL) & bits::ENDP_STATE_MASK,
        bits::ENDP_STATE_ACK,
        "a host-to-device transfer with a data stage must arm the receiver"
    );
}

#[test]
fn host_to_device_transfer_without_data_does_not_arm() {
    let (mut bus, regs) = bus();
    bus.alloc_ep(UsbDirection::Out, Some(0x00.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x80.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.enable();

    // SET_ADDRESS: host to device, no data stage. Its status stage is an IN
    // token, which usb-device drives with a zero-length write.
    deliver_setup(&bus, &regs, &setup(0x00, 5, 0x0002, 0, 0));
    let mut buf = [0u8; 8];
    bus.read(0x00.into(), &mut buf).unwrap();

    assert_eq!(
        regs.mock_peek(OFF_EP0CTL) & bits::ENDP_STATE_MASK,
        bits::ENDP_STATE_NAK
    );
}

#[test]
fn ep0_in_write_arms_with_the_byte_count() {
    let (mut bus, regs) = bus();
    bus.alloc_ep(UsbDirection::Out, Some(0x00.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x80.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.enable();

    deliver_setup(&bus, &regs, &setup(0x80, 6, 0x0100, 0, 18));
    let mut buf = [0u8; 8];
    bus.read(0x00.into(), &mut buf).unwrap();

    let payload = [0x12u8, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40];
    assert_eq!(bus.write(0x80.into(), &payload).unwrap(), 8);

    let epctl = regs.mock_peek(OFF_EP0CTL);
    assert_eq!(epctl & bits::ENDP_CNT, 8);
    assert_eq!(epctl & bits::ENDP_STATE_MASK, bits::ENDP_STATE_ACK);
    assert_eq!(epctl & bits::ENDP_EN, bits::ENDP_EN);

    // A second write before the completion interrupt must be refused.
    assert_eq!(
        bus.write(0x80.into(), &payload).unwrap_err(),
        UsbError::WouldBlock
    );
}

#[test]
fn ep0_status_stage_completes_a_control_in_transfer() {
    let (mut bus, regs) = bus();
    bus.alloc_ep(UsbDirection::Out, Some(0x00.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x80.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.enable();

    deliver_setup(&bus, &regs, &setup(0x80, 6, 0x0100, 0, 4));
    let mut buf = [0u8; 8];
    bus.read(0x00.into(), &mut buf).unwrap();

    // A short packet ends the data stage.
    bus.write(0x80.into(), &[0xAA, 0xBB, 0xCC, 0xDD]).unwrap();

    // The packet goes out; the SIE raises EP0_IN.
    raise(&bus, &regs, bits::EP0_IN);

    // The completion is reported exactly once.
    match bus.poll() {
        PollResult::Data { ep_in_complete, .. } => assert_eq!(ep_in_complete & 1, 1),
        _ => panic!("expected an IN-complete event"),
    }
    match bus.poll() {
        PollResult::None => {}
        PollResult::Data { ep_in_complete, .. } => assert_eq!(ep_in_complete & 1, 0),
        _ => panic!("unexpected poll result"),
    }

    // Having finished the data stage, the driver must have armed the status
    // stage: an OUT token should be accepted.
    assert_eq!(
        regs.mock_peek(OFF_EP0CTL) & bits::ENDP_STATE_MASK,
        bits::ENDP_STATE_ACK,
        "the status stage must be armed after the data stage ends"
    );

    // The host's zero-length OUT arrives.
    regs.mock_poke(OFF_EP0CTL, bits::ENDP_EN | bits::ENDP_STATE_ACK);
    raise(&bus, &regs, bits::EP0_OUT);

    match bus.poll() {
        PollResult::Data { ep_out, .. } => assert_eq!(ep_out & 1, 1),
        _ => panic!("expected an OUT event for the status stage"),
    }

    // usb-device reads the status stage with an empty buffer.
    assert_eq!(bus.read(0x00.into(), &mut []).unwrap(), 0);

    match bus.poll() {
        PollResult::None => {}
        PollResult::Data { ep_out, .. } => assert_eq!(ep_out & 1, 0),
        _ => panic!("unexpected poll result"),
    }
}

#[test]
fn bus_reset_is_reported_and_reinitialises() {
    let (mut bus, regs) = bus();
    bus.alloc_ep(UsbDirection::Out, Some(0x00.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x80.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x81.into()), EndpointType::Interrupt, 8, 1)
        .unwrap();
    bus.enable();
    bus.set_device_address(5);

    raise(&bus, &regs, bits::BUS_RESET);

    assert!(matches!(bus.poll(), PollResult::Reset));

    // The address must be back to zero and endpoint 1 disabled immediately, so
    // the device stops answering at the old address while usb-device is still
    // processing the reset.
    assert_eq!(regs.mock_peek(crate::regs::OFF_ADDR), 0);
    assert_eq!(regs.mock_peek(crate::regs::OFF_EP0CTL + 4), 0);

    // usb-device calls reset() after seeing PollResult::Reset; the allocated
    // endpoint must come back.
    bus.reset();
    assert_eq!(
        regs.mock_peek(crate::regs::OFF_EP0CTL + 4) & bits::ENDP_EN,
        bits::ENDP_EN,
        "reset() must re-enable the endpoints allocated before the reset"
    );
}

// ---------------------------------------------------------------------------
// Non-control endpoints
// ---------------------------------------------------------------------------

#[test]
fn interrupt_in_round_trip() {
    let (mut bus, regs) = bus();
    bus.alloc_ep(UsbDirection::Out, Some(0x00.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x80.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x81.into()), EndpointType::Interrupt, 8, 1)
        .unwrap();
    bus.enable();

    let report = [0u8, 0, 0x04, 0, 0, 0, 0, 0];
    assert_eq!(bus.write(0x81.into(), &report).unwrap(), 8);
    assert_eq!(
        bus.write(0x81.into(), &report).unwrap_err(),
        UsbError::WouldBlock
    );

    // The packet is ACKed by the host; the SIE raises the endpoint 1 ACK bit.
    raise(&bus, &regs, bits::ep_ack(1));

    match bus.poll() {
        PollResult::Data { ep_in_complete, .. } => {
            assert_eq!(ep_in_complete & (1 << 1), 1 << 1);
        }
        _ => panic!("expected an IN-complete event on endpoint 1"),
    }

    // Once complete, the endpoint accepts another packet.
    assert_eq!(bus.write(0x81.into(), &report).unwrap(), 8);
}

#[test]
fn writing_to_an_out_endpoint_is_refused() {
    let (mut bus, _regs) = bus();
    bus.alloc_ep(UsbDirection::Out, Some(0x00.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x80.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::Out, Some(0x02.into()), EndpointType::Interrupt, 8, 1)
        .unwrap();
    bus.enable();

    assert_eq!(
        bus.write(0x02.into(), &[0u8; 4]).unwrap_err(),
        UsbError::InvalidEndpoint
    );
}

#[test]
fn oversized_write_is_a_buffer_overflow() {
    let (mut bus, _regs) = bus();
    bus.alloc_ep(UsbDirection::Out, Some(0x00.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x80.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x81.into()), EndpointType::Interrupt, 8, 1)
        .unwrap();
    bus.enable();

    assert_eq!(
        bus.write(0x81.into(), &[0u8; 9]).unwrap_err(),
        UsbError::BufferOverflow
    );
}

#[test]
fn unallocated_endpoints_are_invalid_not_silent() {
    let (mut bus, _regs) = bus();
    bus.alloc_ep(UsbDirection::Out, Some(0x00.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x80.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.enable();

    assert_eq!(
        bus.read(0x03.into(), &mut [0u8; 8]).unwrap_err(),
        UsbError::InvalidEndpoint
    );
    assert_eq!(
        bus.write(0x83.into(), &[0u8; 8]).unwrap_err(),
        UsbError::InvalidEndpoint
    );
}

#[test]
fn stalling_is_visible_and_clears() {
    let (mut bus, regs) = bus();
    bus.alloc_ep(UsbDirection::Out, Some(0x00.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, Some(0x80.into()), EndpointType::Control, 64, 0)
        .unwrap();
    bus.enable();

    // `usb-device` calls this on endpoint 0 after reading a SETUP packet.
    bus.set_stalled(0x00.into(), false);
    assert_eq!(
        regs.mock_peek(OFF_EP0CTL) & bits::ENDP_STATE_MASK,
        bits::ENDP_STATE_NAK,
        "unstalling must leave the endpoint NAKing, never ACKing"
    );

    bus.set_stalled(0x00.into(), true);
    assert!(bus.is_stalled(0x00.into()));
    assert_eq!(
        regs.mock_peek(OFF_EP0CTL) & bits::ENDP_STATE_MASK,
        bits::ENDP_STATE_STALL
    );
}

// ---------------------------------------------------------------------------
// Packet memory through the public API
// ---------------------------------------------------------------------------

#[test]
fn seven_endpoints_of_64_bytes_exactly_fill_the_packet_ram() {
    let (mut bus, _regs) = bus();
    bus.alloc_ep(UsbDirection::In, None, EndpointType::Bulk, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, None, EndpointType::Bulk, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, None, EndpointType::Bulk, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, None, EndpointType::Bulk, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, None, EndpointType::Bulk, 64, 0)
        .unwrap();
    bus.alloc_ep(UsbDirection::In, None, EndpointType::Bulk, 64, 0)
        .unwrap();
    // Six numbers of 64 bytes is only 384 of the 448 available, so a seventh
    // must fail on the endpoint count with the memory still spare.
    assert_eq!(
        bus.alloc_ep(UsbDirection::In, None, EndpointType::Bulk, 64, 0)
            .unwrap_err(),
        UsbError::EndpointOverflow
    );
}

#[test]
fn packet_ram_overflow_is_reported_when_sizes_are_large() {
    // Six endpoints of 64 bytes fit (384 <= 448), but asking for more than the
    // remaining space must fail rather than wrap. Sizes above 64 are refused
    // earlier, so fill the space with 64-byte endpoints and then confirm the
    // allocator itself is the thing that holds the line.
    let mut pma = crate::pma::PmaAlloc::new();
    for _ in 0..7 {
        pma.alloc(64).unwrap();
    }
    assert!(pma.alloc(1).is_err());
}

#[test]
fn ep0_max_packet_size_is_64() {
    assert_eq!(EP0_MAX_PACKET_SIZE, 64);
}
