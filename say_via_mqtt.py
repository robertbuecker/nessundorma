#!/usr/bin/env python3
# coding: utf-8

import asyncio_mqtt as mqtt
import simpleaudio as sa
import time
import json
import numpy as np
import sys
from warnings import warn
import asyncio
from timed_message_dispatcher import TimedMessageDispatcher
import json
import asyncio_mqtt as mqtt
from putzini_config import PutziniConfig
import logging

logger = logging.getLogger(__name__)

async def listen_for_beamer(mqtt_client):
    async with mqtt_client as client:
        async with client.filtered_messages('beamer') as messages:
            await client.subscribe('beamer')
            logger.info('Started listening.')
            async for message in messages:
                text = json.loads(message.payload.decode())
                if 'Regie' in text and (text["Regie"] is not None):
                    await asyncio.create_subprocess_exec('say', text["Regie"])
                         
async def main():
    import asyncio_mqtt as mqtt
    from putzini_config import PutziniConfig
    config = PutziniConfig()
    client = mqtt.Client(config.mqtt_broker)
    asyncio.ensure_future(listen_for_beamer(mqtt_client=client))

    while True:
        await asyncio.sleep(1)

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    loop = asyncio.get_event_loop()      
    # loop.set_debug(True)
    loop.run_until_complete(main())
    loop.close()