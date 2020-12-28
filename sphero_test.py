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
    await my_sphero.connect()
    await my_sphero.wake()

    # roll in a square
    # for i in range(4):
    #     await my_sphero.roll(speed=150,heading=90*i)
    #     await asyncio.sleep(1)

    # disco
    for _ in range(100):
        await my_sphero.setLEDColor(red=randrange(255), \
            green=randrange(255),blue=randrange(255))

    # stop rolling
    await my_sphero.roll(speed=0,heading=0)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.set_debug(True)
    loop.run_until_complete(run())