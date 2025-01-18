import cocotb
from cocotb.triggers import RisingEdge, FallingEdge, Timer

async def generate_clock(dut):
    while(1):
        dut.clk.value = 0
        await Timer(1, units="ns")
        dut.clk.value = 1
        await Timer(1, units="ns")

    
def load_rom(filename):
    with open(filename, mode='rb') as file:
        return bytearray(file.read())
    println(f"Error opening {filename}")
    return bytearray(2**16)


async def do_memory(dut, memory):
    while(1):
        await RisingEdge(dut.clk)
        #read memory is non valid until 2nd half of cycle
        dut.data_i.value = 0xff  

        await FallingEdge(dut.clk)
        rw = dut.RW.value
        addr = int(dut.addr.value)
        # dut.data_i.value = 0xff
        if(rw == 0):
            memory[addr] = dut.dor.value
        else:
            dut.data_i.value = memory[addr]


@cocotb.test()
async def run_bin(dut):

    memory = load_rom("../test/testroms/build/6502_functional_test.bin")

    dut.clk.value = 0
    dut.data_i.value = 0
    dut.READY.value = 1
    dut.SV.value = 1
    dut.NMI.value = 0
    dut.IRQ.value = 0
    dut.rst.value = 1

    await cocotb.start(generate_clock(dut))  # run the clock "in the background"
    await cocotb.start(do_memory(dut, memory))  # run the clock "in the background"

    await Timer(5, units="ns")  # wait a bit
    await RisingEdge(dut.clk)
    dut.rst.value = 0

    cnt = 0
    while(1):
        cnt = cnt + 1
        if (cnt%10000 == 0) :
            print(cnt)
        if (dut.ip.value == 0x336D):
            dut._log.info("SUCCESS!")
            pass



    # for i in range(7):
    #     await RisingEdge(dut.clk)
    #     if(dut.RW.value == 1):
    #         dut._log.info("ram[%04x] => %02x", dut.addr.value, dut.data_i.value)
    #     else:
    #         dut._log.info("ram[%04x] <= %02x", dut.addr.value, dut.dor.value)
