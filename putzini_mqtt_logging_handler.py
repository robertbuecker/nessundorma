"""JUST AN EXAMPLE"""

import logging
import yaml
import asyncio
import asyncio_mqtt

class PutziniMqttLoggingHandler(logging.Handler):
    """
        A handler class which writes logging records, appropriately formatted,
        to a MQTT server to a topic.
        """
    def __init__(self, mqtt_client, topic):
        logging.Handler.__init__(self)
        self.mqtt_client = mqtt_client
        self.topic = topic
        self.story_step = "StoryStep"
        #asyncio.ensure_future(self._story_step_listener)
   
    #async def _story_step_listener():
    #    async with client.filtered_messages("simulation/state") as messages:
    #        await client.subscribe("simulation/state")
    #        async for message in messages:
    #            msg = json.loads(message.payload.decode("utf-8"))
    #            self.story_step = msg["current step"]

    def emit(self, record):
        """
                Publish a single formatted logging record to a broker, then disconnect
                cleanly.
                """
        record.story_step = self.story_step
        msg = self.format(record)
        if not 'pending publish calls' in msg.lower():
            asyncio.ensure_future(self.mqtt_client.publish(self.topic, msg)) 
        else:
            print('Blocked MQTT logging:', msg)

async def main():
    topic = 'putzini/logs'
    async with asyncio_mqtt.Client("172.31.1.150") as client:
        await client.publish("putzini/logs", "hallo")

        # Create and configure a logger instance
        logger = logging.getLogger('')
        myHandler = PutziniMqttLoggingHandler(client, topic)
        myHandler.setLevel(logging.INFO)
        myHandler.setFormatter(logging.Formatter('%(story_step)s - %(levelname)s: %(message)s'))
        logger.addHandler(myHandler)
        
        logging.info("test lalala info")
        logging.error("test lalala info")
        await asyncio.sleep(10)



if __name__ == '__main__':        
    loop = asyncio.get_event_loop()
    loop.set_debug(True)
    loop.run_until_complete(main())
