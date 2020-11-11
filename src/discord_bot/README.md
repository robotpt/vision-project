README
======

This package creates a discord bot that is used for getting simple feedback during a human robot interaction.

The `discord_bot` script require two environmental variables to be set:
- `DISCORD_TOKEN`: the token for a Discord bot (see [here](https://realpython.com/how-to-make-a-discord-bot-python/) for instructions)
- `DISCORD_CHANNEL_ID`: the channel ID (an integer, like `771882722753773581`) where the bot should post messages 

This `discord_bot` has two commands (other than `help`) and you `@` the discord bot to send it commands.
The two commands are `status` and `pick`.
If the bot has received a ROS message giving it options, it will announce that it is ready and you can use the `pick` command
to start dialog where it will give you possible choices to pick.
Note, if you take to long, the bot will timeout (20 seconds). If this happens, just use the pick command again when you are ready.

From the ROS perspective the `discord_bot` has two topic channels that it uses:
- `annotator/choices`: a subscriber for the choices that the bot will suggest. This topic uses `String` messages where choices are separated by semicolons.
  For example: `Choice 1;Choice 2;My default option` will yield three options: `Choice 1`, `Choice 2`, and `My default option`.
- `annotator/pick`: a publisher that shows the choice that has been selected through discord by the `pick` command. This topic also uses a string.

TODO
----
- [ ] Make sure that this can be on continuously.
