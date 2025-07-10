import speech_recognition as sr
import pyttsx3
import subprocess
import re
import sys
import ollama

# === Settings ===
MODEL_NAME = "llama3"
EXPERIMENT_SCRIPT = r"D:\PythonProjects\autohiam\experiment.py"

# === Setup ===
engine = pyttsx3.init()          # already in your code
engine.setProperty("voice", "HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Speech\\Voices\\Tokens\\TTS_MS_EN-GB_HAZEL_11.0")

state = "GREETING"
pending_cycles = None

def speak(text):
    print("🔈", text)
    engine.say(text)
    engine.runAndWait()

def listen(timeout=5, phrase_time_limit=8):
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("🎙️ Listening...")
        recognizer.adjust_for_ambient_noise(source, duration=0.5)
        try:
            audio = recognizer.listen(source, timeout=timeout, phrase_time_limit=phrase_time_limit)
        except sr.WaitTimeoutError:
            return ""
    try:
        return recognizer.recognize_google(audio)
    except:
        return ""

def chat_with_llama(user_input, system_prompt):
    response = ollama.chat(
        model=MODEL_NAME,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_input}
        ]
    )
    return response['message']['content'].strip()

# === Main ===
speak("Hi, my name is AUTOHIAM. I am an automatic robotic system for hydrogel infusion additive manufacturing. "
      "Can I help you with anything today?")

while True:
    user_input = listen()
    if not user_input:
        continue
    print(f"🗣️ You: {user_input}")

    if state == "GREETING":
        if re.search(r"\brun\b.*\bhiam\b|\bexperiment\b", user_input, re.IGNORECASE):
            speak("Sure. Could you please specify how many cycles you want to run?")
            state = "AWAIT_CYCLE_NUMBER"
            continue

        # Otherwise, free conversation
        response = chat_with_llama(
            user_input,
            "You are AUTOHIAM, a friendly lab assistant. Chat naturally. Do not execute anything."
        )
        speak(response)

    elif state == "AWAIT_CYCLE_NUMBER":
        # Try to find a number
        match = re.search(r"\b(\d+)\b", user_input)
        if match:
            pending_cycles = int(match.group(1))
            speak(f"Okay. Starting the experiment with {pending_cycles} cycles.")
            subprocess.Popen([
                sys.executable, EXPERIMENT_SCRIPT, "--cycles", str(pending_cycles)
            ])
            state = "GREETING"
            speak("Experiment launched. Let me know if you need anything else.")
        else:
            speak("Sorry, I didn't catch the number. How many cycles should I run?")
