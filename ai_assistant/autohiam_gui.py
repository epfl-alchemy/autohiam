import queue
import re
import subprocess
import sys
import threading
import time
import tkinter as tk

import ollama
import pyttsx3
import speech_recognition as sr

MODEL_NAME = "llama3"
EXPERIMENT_SCRIPT = r"D:\PythonProjects\autohiam\experiment.py"

engine = pyttsx3.init()
engine.setProperty("voice",
                   # "HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Speech\Voices\Tokens\TTS_MS_EN-US_ZIRA_11.0")
                   "HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Speech\\Voices\\Tokens\\TTS_MS_EN-GB_HAZEL_11.0")

# ── GUI SET-UP ───────────────────────────────────────────
root = tk.Tk()
root.configure(bg="black")
root.attributes("-fullscreen", True)
tk.Label(root, text="AUTOHIAM", font=("Helvetica", 68, "bold"),
         fg="white", bg="black").pack(pady=50)

label_status = tk.Label(root, text="Press SPACE to speak",
                        font=("Helvetica", 18), fg="lightgray", bg="black")
label_status.pack(pady=20)

label_user = tk.Label(root, font=("Helvetica", 20), fg="cyan",
                      bg="black", wraplength=1000)
label_user.pack(pady=10)

label_bot = tk.Label(root, font=("Helvetica", 20), fg="lightgreen",
                     bg="black", wraplength=1000)
label_bot.pack(pady=10)

label_exp = tk.Label(root, font=("Consolas", 18), fg="#ffa500",
                     bg="black", justify="left", anchor="w", wraplength=1200)
label_exp.pack(pady=20)

# thread-safe queue for experiment output
exp_queue: "queue.Queue[str]" = queue.Queue()


# ── SPEAK / LISTEN HELPERS ───────────────────────────────
def speak(txt: str):
    # Display the output of the speaking content
    # label_bot.config(text=txt);
    root.update()
    engine.say(txt);
    engine.runAndWait()


def listen(timeout=30, limit=15):
    r = sr.Recognizer()
    with sr.Microphone() as src:
        label_status.config(text="🎙️ Listening…");
        root.update()
        r.adjust_for_ambient_noise(src, 0.5)
        try:
            audio = r.listen(src, timeout=timeout, phrase_time_limit=limit)
        except sr.WaitTimeoutError:
            return ""
    try:
        return r.recognize_google(audio)
    except:
        return ""


# ── EXPERIMENT LAUNCH & STREAM ───────────────────────────
def run_experiment(cycles: int):
    """Launch experiment.py and stream its stdout to the GUI, then notify on completion."""
    proc = subprocess.Popen(
        [sys.executable, EXPERIMENT_SCRIPT, "--cycles", str(cycles)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1, universal_newlines=True
    )

    def reader():
        for line in proc.stdout:
            exp_queue.put(line.rstrip())
        proc.stdout.close()
        proc.wait()
        # Speak after process has completed
        speak("Hi! I'm happy to tell you I have finished the experiment.")

    threading.Thread(target=reader, daemon=True).start()


# periodic GUI update for experiment output
def poll_exp_output():
    try:
        while True:
            line = exp_queue.get_nowait()
            current = label_exp.cget("text")
            label_exp.config(text=current + line + "\n")
    except queue.Empty:
        pass
    root.after(100, poll_exp_output)  # keep polling


poll_exp_output()  # start polling loop


# ── MAIN INTERACTION LOGIC ───────────────────────────────
def handle_voice():
    user = listen()
    if not user:
        label_status.config(text="Press SPACE to speak");
        return

    # Hide user speech on GUI
    # label_user.config(text=f"🗣️ {user}")

    # 1) ask for cycles
    if re.search(r"\brun\b.*(auto\s*hiam|autohiam|hiam)\b|\bexperiment\b", user, re.I):
        speak("Sure. How many cycles would you like to run?")
        root.update();
        time.sleep(0.5)
        cycles_str = listen(limit=10)
        label_user.config(text=f"🗣️ {cycles_str}")
        m = re.search(r"\b(\d+)\b", cycles_str or "")
        if m:
            cycles = int(m.group(1))
            speak(f"Starting the experiment with {cycles} cycles.")
            run_experiment(cycles)
        else:
            speak("Sorry, I didn't catch a number.")
    else:
        # 2) normal chat through LLaMA
        reply = ollama.chat(
            model=MODEL_NAME,
            messages=[
                {"role": "system",
                 "content": "You are AUTOHIAM, a friendly lab assistant for Hydrogel Infusion Additive Manufacturing "
                            "experiment. You chat with users in a short and concise way. You are always ready to ask "
                            "how you can help with the HIAM experiment today. You only care about the number of "
                            " you need to run and nothing else."},
                {"role": "user", "content": user}
            ])['message']['content']
        speak(reply)

    label_status.config(text="Press SPACE to speak")


# ── HOTKEYS ──────────────────────────────────────────────
root.bind("<space>", lambda e: threading.Thread(target=handle_voice).start())
root.bind("<Escape>", lambda e: root.destroy())

# ── STARTUP ──────────────────────────────────────────────
speak(
    "Hi, I’m AUTOHIAM. I am an automatic robotic system for Hydrogel Infusion Additive Manufacturing experiments. Can I help you with anything today?")
root.mainloop()
