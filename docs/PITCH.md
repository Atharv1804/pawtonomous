# Demo script & pitch: Pawtonomous, "Jarvis", my AI robot pet

About 5 minutes. **Bold** = say it out loud to Jarvis (always start with "Hey Jarvis", then wait for the chime).
*Italics* = what you do. > Quotes = what you say to the judges.
Keep your **iPhone in your hand**: open the Jarvis Remote, tap **TALK**, speak, tap **DONE**. Its buttons are a backup for every trick.

---

## 0. Before you're called up
- Mac open, Astrill on, LM Studio running, same WiFi as the robot. Pi + motor battery + ESP32 charged.
- Robot on the floor about 2 m in front of you, **facing you**. Screen shows the eyes and the "FOLLOW ME" button.
- Optional: say "reset friends" to me beforehand so the friends counter starts at 0.

## 1. Walk-on: Jarvis follows you
**"Hey Jarvis, follow me."** *(or tap FOLLOW ME on his screen)*
*Walk slowly backwards towards the stage, facing him. He follows your face and stays about 1 m behind.*
*At the stage: **"Hey Jarvis, stop."** (or tap the red STOP). Pick him up and put him on the table facing the judges.*

## 2. Hello
**"Hey Jarvis, say hi to the judges."**
*(a paw waves on his screen: "Hi everyone, I'm Jarvis, Atharv's pet! It's really nice to meet you all!")*

> "Good boy. This is Jarvis, my robot pet. He just heard me, followed me up here, and said hello to
> you, and none of it needed a paid AI service."

**"Hey Jarvis, sit."** *(a little shuffle, then he closes his eyes and dozes off: z z z)*

## 3. Why I built him
> "Most smart devices are stuck in one place and stuck in the cloud. A smart speaker can't come to
> you, can't see who's in the room, and stops working when the internet or the subscription does.
> I wanted a companion that **moves, sees, listens and remembers**, runs on hardware I already own,
> and keeps what its camera sees **private**. So I built Jarvis."

## 4. His body
> "Jarvis's body runs on a Raspberry Pi 5 with **ROS 2**, the Robot Operating System, in Docker.
> Four DC motors are driven by two BTS7960 motor drivers and an **ESP32** microcontroller that talks to
> the Pi over WiFi. Three ultrasonic sensors on an Arduino act like whiskers. He also has a camera,
> microphones, a speaker, a touchscreen face, and his own battery pack.
>
> Safety is built in at every level. A 'safety reflex' in ROS stops the motors if something gets too
> close or if commands stop arriving: he halts within half a second, even if the AI crashes. The
> motor controller only accepts commands signed with a secret key, and only my Telegram account can
> control him."

## 5. His brain (two brains)
> "Jarvis actually has **two brains**.
>
> The **fast brain** is his reflexes. It recognises faces and people about ten times a second using
> OpenCV models: YuNet for faces, SFace to know *whose* face it is, and NanoDet to find people. Commands
> like 'follow me', 'sit' or 'stop' are recognised instantly, without any language model, so he
> reacts in about a second.
>
> The **thinking brain** is an AI agent, **Hermes Agent** by Nous Research. It controls his body through
> **22 tools** over the **Model Context Protocol**: drive, follow, find a person, look, listen, speak,
> check his battery, take photos, learn faces.
>
> The AI runs **locally**:
> - **gpt-oss-20b**, OpenAI's open-weight model, in LM Studio, is his main reasoning model,
> - **Qwen 3.5** (a small vision model) in Ollama describes what he sees in under a second,
> - **Whisper** turns speech into text on the Apple GPU, **Piper** is his voice, and **openWakeWord**
>   listens for 'Hey Jarvis'.
>
> If the local models are busy, he can fall back to **cloud models**: **Google Gemini 3 Flash** and
> **Gemini 3.1 Flash Lite**, or free models on **OpenRouter** like NVIDIA Nemotron. By default, nothing
> leaves the room.
>
> The Pi on its own overheated running all of this, so I split the work. The Pi is his **body**: camera,
> motors, sensors, speaker. My laptop is his **brain**, linked through one encrypted SSH tunnel. That
> took the Pi from about 85% CPU and 63 °C down to about 15% and 50 °C."

## 6. Treat time (your phone rings)
*Your phone rings (set a timer for this moment).*
> "Sorry, one second… oh, it's my pet. He wants a treat."

*Bend down:* **"Hey Jarvis, here's your treat."**
*(his eyes spin happily, "Yummy! Thank you!", a happy little step, then sleepy eyes)*

> "That's his personality layer: expressions, a mouth that moves when he talks, and a status line at
> the bottom so you always know what he's doing."

## 7. How he sees (pick him up)
*Pick him up and point him at yourself and the judges. Tap the **dot** in the bottom-right corner*
*(or say **"Hey Jarvis, show them what you see"**).*

> "This is what Jarvis sees right now. My face is labelled **'Atharv'**: he learned me from a single
> photo I sent him on Telegram, and he keeps learning me from his own camera. Your faces show a
> **question mark**, because he doesn't know you yet.
>
> When I'm with him, the people I spend time with go into a **friends list**: you can see it counting
> up at the top, 'friends met'. He stores a face *signature*, not a photo, so camera images never leave
> this desk. And because he runs on Hermes Agent, he has long-term memory for our conversations too."

*Tap the dot again to go back to his face.*

## 8. What was hard
> "The hardest problems weren't the AI. I built this in China, where GitHub, Docker Hub, Telegram and
> Google are blocked, so I routed everything through mirrors and a tunnel. I burned out one ESP32, found
> dead pins on the Arduino, and one motor driver can't run backwards, so I rewrote the firmware to
> steer with what works. The local AI was too slow at first. By profiling it, I cut a question from
> about 15 seconds to about 6, and simple commands to under one second."

## 9. Selfie
*Hold him at arm's length, both of you in view:* **"Hey Jarvis, take a selfie."**
*(or tap the small ring next to the dot)* *("Say cheese!": his screen shows what he is about to photograph, 3-2-1,
then the photo. Tap **< BACK** during the countdown to cancel.)*

> "And because he runs on Hermes Agent, that selfie is already on my phone, in Telegram."
*(show your phone)*

## 10. Close
> "So that's Jarvis: a robot pet that sees, listens, talks, follows me, remembers his friends and
> explores on his own, built from affordable off-the-shelf parts and open-source AI that runs locally.
> Next I want to add wheel encoders and a front depth sensor for proper navigation, let him map rooms,
> and build him a charging dock he can drive back to by himself."

**"Hey Jarvis, say bye."** *(a paw waves: "Bye everyone! Thank you for meeting me!")*
> "Thank you!"

---

## Cheat sheet
| Want | Say ("Hey Jarvis, …") | Or tap |
|---|---|---|
| follow | follow me | FOLLOW ME (bottom left) |
| stop | stop | red STOP (top) |
| greet / goodbye | say hi to the judges / say bye | – |
| sit | sit | – |
| treat | here's your treat | – |
| camera view | show them what you see | dot (bottom right) |
| selfie | take a selfie | small ring next to the dot (< BACK cancels) |

## If something goes wrong
- **He doesn't react:** say it again closer to your iPhone or the MacBook, and wait for the chime.
- **He won't follow:** face him and stand about 2 m away in good light, or carry him: "He's shy in front of judges."
- **He doesn't recognise you:** "He's still learning: I only gave him one photo."
- **He moves when he shouldn't:** tap STOP or say "Hey Jarvis, stop".
