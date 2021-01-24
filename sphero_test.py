import asyncio

from random import randrange
from sphero_mini import SpheroMini

async def run():
    # mac address of sphero mini
    address = (
        "CE:FE:51:E5:40:06"
    )

    # connect to sphero mini
    my_sphero = SpheroMini(address)
    try:
        await my_sphero.connect()

        # wake sphero
        await my_sphero.wake()
        await my_sphero.getAcknowledgement("wake")

        # roll in a square
        # for i in range(4):
        #     await my_sphero.roll(speed=150,heading=90*i)
        #     await asyncio.sleep(1)

        # disco
        for _ in range(100):
            await my_sphero.setLEDColor(red=randrange(255), \
                green=randrange(255),blue=randrange(255))
            await my_sphero.getAcknowledgement("led")

        # battery voltage
        await my_sphero.getVoltage()
        await my_sphero.getAcknowledgement("voltage")
        await asyncio.sleep(1)

    finally:
        await my_sphero.disconnect()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.set_debug(True)
    loop.run_until_complete(run())
