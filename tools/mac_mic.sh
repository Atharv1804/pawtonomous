#!/bin/bash
# Let Jarvis hear through the MacBook mic ONLY while this terminal window is open.
# Close the window (or press Ctrl+C) and the Mac mic is released.
export HERMES_API_KEY_FILE=$HOME/robot_ws/.hermes_api_key HERMES_API_URL=http://127.0.0.1:18642 \
       MAC_AI_URL=http://127.0.0.1:8768 PET_URL=http://127.0.0.1:8766 PI_IO_URL=http://127.0.0.1:18767 \
       VOICE_PTT=1 ROBOT_SPEAK_VIA=pi_io
echo "Jarvis is listening on the Mac mic. Close this window or press Ctrl+C to stop."
exec "$HOME/robot_ws/mac/.venv/bin/python" -u "$HOME/robot_ws/tools/voice_loop.py"
