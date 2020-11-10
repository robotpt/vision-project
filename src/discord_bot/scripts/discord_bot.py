#!/usr/bin/env python3.8

import asyncio
import discord
from discord.ext import commands
import os
import rospy
import typing

from std_msgs.msg import String


TOKEN = os.environ["DISCORD_TOKEN"]
CHANNEL_ID = int(os.environ["DISCORD_CHANNEL_ID"])


class MyBot(commands.Bot):

    async def on_ready(self):
        channel = await self.fetch_channel(CHANNEL_ID)
        await channel.send(f"I'm ON and ready!")


class RosDiscordCog(commands.Cog):
    def __init__(self, bot: commands.Bot):
        rospy.init_node("ros_discord_bot")
        self._bot = bot
        self._choices = None
        self._is_new_choices = False

        self._pick_publisher = rospy.Publisher("annotators/pick", String, queue_size=1)
        self._choices_subscriber = rospy.Subscriber(
            "annotators/choices",
            String,
            callback=self._choices_subscriber_callback,
            queue_size=1,
        )
        self._background_task = self._bot.loop.create_task(
            self._send_message_on_new_choices()
        )

    async def _send_message_on_new_choices(self):
        await self._bot.wait_until_ready()
        channel = await self._bot.fetch_channel(CHANNEL_ID)
        while not self._bot.is_closed():
            if self._is_new_choices:
                await channel.send("I'm ready for you to pick!")
                self._is_new_choices = False
            await asyncio.sleep(5)

    def _choices_subscriber_callback(self, msg: String):
        data = msg.data.strip()
        if data == '':
            rospy.logwarn("Choices string was empty!")
            return
        self._choices = data.split(';')
        self._is_new_choices = True
        rospy.loginfo("Choices received!")

    @commands.Cog.listener()
    async def on_member_join(self, member):
        channel = member.guild.system_channel
        if channel is not None:
            await channel.send("Welcome {0.mention}.".format(member))

    @commands.command(help="Check to see I am ready for you to pick from choices")
    async def status(self, ctx):
        if self._choices is not None:
            message = "Ready for you to pick!"
        else:
            message = "Not ready for you to pick."
        await ctx.send(message)

    @commands.command(
        name="pick", help="Pick from a set of choices when they're available"
    )
    async def ask_to_pick(self, message):

        if self._choices is None:
            await message.channel.send("Oops, nothing for you to pick.")
            return

        await message.channel.send(self._get_options_str(self._choices))

        def is_correct(m):
            return (
                m.author == message.author
                and m.content.isdigit()
                and 1 <= int(m.content) <= len(self._choices)
            )

        try:
            choice: discord.Message = await self._bot.wait_for(
                "message", check=is_correct, timeout=20.0
            )
        except asyncio.TimeoutError:
            return await message.channel.send(
                "Sorry, you took too long. Please enter the command."
            )

        choice_idx = int(choice.content)
        self._send_choice(self._choices[choice_idx - 1])
        await message.channel.send("Great, thank you.")
        self._choices = None

    def _send_choice(self, choice: str):
        if not rospy.is_shutdown():
            self._pick_publisher.publish(choice)
            rospy.loginfo(f'"Picked choice at {rospy.get_time()}: {choice}.')

    @staticmethod
    def _get_options_str(options: typing.List[str]) -> str:
        out = "Please pick one of the following:\n"
        for idx, option in enumerate(options, 1):
            out += f"\t{idx}) {option}\n"
        return out


if __name__ == "__main__":

    b = MyBot(command_prefix=commands.when_mentioned)
    b.add_cog(RosDiscordCog(b))
    b.run(TOKEN)
