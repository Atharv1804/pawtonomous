# Pitch: Pawtonomous, "Jarvis", my AI robot pet

About 4-5 minutes. **Bold** = say it to the robot (always start with "Hey Jarvis"). *Italics* = stage direction.

---

### 1. Walk-on (the robot follows you)
*Before you're introduced, stand 2 m in front of the robot, facing it.*
**"Hey Jarvis, follow me."** *Walk slowly, in a straight line, towards the stage. It follows your body,
even from behind. Pick it up at the stage, put it on the table facing the judges.*

### 2. Hello
**"Hey Jarvis, say hi to the judges."**
*(paw waves on screen: "Hi everyone, I'm Jarvis, Atharv's pet! It's really nice to meet you all!")*

> "Good boy. This is Jarvis, my robot pet, and everything you just saw happened *on this desk*:
> he heard me, understood me, followed me here and answered, without a single paid AI service."

**"Hey Jarvis, sit."** *(he shuffles and closes his eyes)*

### 3. The idea
> "Most 'smart' devices are stuck in one place and stuck in the cloud. A smart speaker can't come to
> you, can't see who's in the room, and stops working when the internet or a subscription does.
> I wanted something different: a companion that **moves, sees, listens and remembers**, that runs on
> hardware I already own, and that keeps what its camera sees **private**. So I built Jarvis."

### 4. How I built the body
> "Jarvis's body is a Raspberry Pi 5 running **ROS 2**, the Robot Operating System, inside Docker. Its
> legs are four DC motors driven by two BTS7960 motor drivers and an **ESP32** microcontroller that
> talks to the Pi over WiFi. His whiskers are three ultrasonic sensors on an Arduino. He has a camera,
> a microphone array, a speaker, a touchscreen face, and a battery pack.
>
> Safety is built in at every level. A ROS 'safety reflex' stops the motors if something gets too
> close or if commands stop arriving: he halts within half a second, even if the AI crashes. The
> motor controller only obeys commands signed with a secret key, so nobody else on the WiFi can drive
> him. And only my Telegram account can talk to him."

### 5. How I built the brain (two brains)
> "The interesting part is that Jarvis has **two brains**.
>
> The **fast brain** handles reflexes: face detection, face recognition and person tracking about ten
> times a second, using OpenCV's YuNet, SFace and NanoDet models. 'Follow me', 'stop' and 'sit' are
> recognised instantly, with no language model involved, so he reacts in under a second.
>
> The **thinking brain** is an AI agent called **Hermes Agent**, from Nous Research. It plans and
> talks, and it controls the body through **22 tools** over the **Model Context Protocol**: drive,
> follow, find a person, look, listen, speak, check the battery, take a photo, learn a face.
>
> All the main AI runs **locally**:
> - **gpt-oss-20b** (OpenAI's open-weight model) in LM Studio as the main reasoning model,
> - **Qwen 3.5** (2B, vision) in Ollama, so he can describe what he sees in under a second,
> - **Whisper** speech-to-text on the Apple GPU, **Piper** for his voice, and **openWakeWord** for 'Hey Jarvis'.
>
> If the local models are busy, he can fall back to **cloud models**: **Google Gemini 3 Flash** and
> **Gemini 3.1 Flash Lite**, and free models on **OpenRouter** like NVIDIA Nemotron. But by default
> nothing leaves the room.
>
> The Pi alone overheated running all of this. So I split it: the Pi is the **body** (camera
> pass-through, motors, sensors, speaker), and my laptop is the **brain**, connected through one
> encrypted SSH tunnel. The Pi went from about 85% CPU and 63 °C down to about 15% and 50 °C."

### 6. Treat time (your phone rings)
*Phone rings.* "Sorry, one second… oh, it's my pet. He wants a treat."
*Bend down:* **"Hey Jarvis, here's your treat."**
*(eyes spin, "Yummy! Thank you!", a happy little step, then sleepy eyes)*
> "That's the personality layer: expressions, a mouth that moves with his voice, and a status line
> so you always know what he's doing."

### 7. How he sees (pick him up)
*Pick him up, tap the dot in the corner of the screen (or say **"Hey Jarvis, show them what you see"**).*
> "This is what Jarvis sees right now. Every face gets a box. My face says **'Atharv'**, because he learned
> me from a single photo I sent him on Telegram. Your faces show a question mark: he doesn't know you
> yet.
>
> When I'm with him, the people I'm interacting with get added to a **friends list**, which you can see
> at the top: 'friends met'. He stores face *signatures*, not photos, so the camera images never leave
> this desk. And because he runs on Hermes Agent, he also has long-term memory for conversations."

*Tap the dot again to go back to his face.*

### 8. What was hard
> "The hardest problems weren't the AI. I built this in China, where GitHub, Docker Hub, Telegram and
> Google are blocked, so I routed everything through mirrors and a tunnel. I burned out one ESP32,
> found dead pins on the Arduino, and one motor driver can't run backwards, so I rewrote the firmware
> to turn using only what works. And the local AI was too slow at first. By profiling it, I cut a robot
> question from about 15 seconds to about 6, and simple commands to under one."

### 9. Selfie
*Hold him up in selfie position:* **"Hey Jarvis, take a selfie."**
*("Say cheese!", 3-2-1, the photo appears on his screen)*
> "And since he runs on Hermes Agent, that selfie is already on my phone, in Telegram."
*(show the phone)*

### 10. Close
> "So that's Jarvis: a robot pet that sees, listens, talks, follows, remembers his friends and
> explores on his own, built from affordable off-the-shelf parts and open-source AI, all running
> locally. Next, I want to add wheel encoders and a front depth sensor for proper navigation, let him
> map rooms and remember where things are, and give him a charging dock he can drive back to by
> himself. Thank you! Jarvis, say bye?"

---

**Backup lines if something fails:**
- He doesn't hear you: get closer to the MacBook and repeat ("Hey Jarvis…", wait for the chime).
- He doesn't move: "He's shy in front of judges," then show the camera view instead.
- He doesn't recognise you: "He's still learning: I gave him just one photo."
