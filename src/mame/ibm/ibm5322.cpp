// license:BSD-3-Clause
// copyright-holders: Hannes Janetzek
/**************************************************************************************************

IBM 5322

-

TODO:

**************************************************************************************************/

#include "emu.h"

#include "cpu/i8085/i8085.h"
#include "machine/ram.h"

namespace {

class ibm5322_state : public driver_device
{
public:
    ibm5322_state(const machine_config &mconfig, device_type type, const char *tag)
        : driver_device(mconfig, type, tag)
        , m_maincpu(*this, "maincpu")
    { }

    void ibm5322(machine_config &config);

private:
    virtual void machine_reset() override;

    required_device<i8085a_cpu_device> m_maincpu;

    void main_map(address_map &map);
    void main_io(address_map &map);

    int sid_r() {
        return m_sid;
    }

    // Probably set by interrupt controller
    // 1 to pass test @ 0x0175
    uint8_t m_sid = 1;
};

void ibm5322_state::main_map(address_map &map)
{
    map(0x0000, 0x7fff).rom().region("ros",0 );
    map(0x8000, 0xbfff).ram();
    map(0xc000, 0xffff).ram(); // TODO banked
}

void ibm5322_state::main_io(address_map &map)
{
    //map(0x41, 0x41).w(FUNC(ibm5322_state::test_pins));

    map(0x41, 0x41).lw8(NAME([this] (offs_t offset, u8 data) {
        logerror("WRITE TEST PINS 0x%02X\n", data);
    }));
}

static INPUT_PORTS_START( ibm5322 )
INPUT_PORTS_END

void ibm5322_state::ibm5322(machine_config &config)
{
    I8085A(config, m_maincpu, XTAL(8'000'000) / 2);
    m_maincpu->set_addrmap(AS_PROGRAM, &ibm5322_state::main_map);
    m_maincpu->set_addrmap(AS_IO, &ibm5322_state::main_io);

    m_maincpu->in_sid_func().set(FUNC(ibm5322_state::sid_r));
}

void ibm5322_state::machine_reset()
{
}

ROM_START( ibm5322 )
    ROM_REGION(0x8000, "ros", 0)
    ROM_LOAD("5322-02-4481186.bin", 0x0000, 0x8000, CRC(61c9866a) SHA1(43f2bed5cc2374c7fde4632948329062e57e994b))
ROM_END

} // namespace

COMP( 1981, ibm5322, 0, 0, ibm5322, ibm5322, ibm5322_state, empty_init, "International Business Machines", "System/23 DataMaster 5322", MACHINE_IS_SKELETON )
