import speech_recognition as sr
import pyttsx3
import subprocess
import ollama

# === Setup: Text-to-speech ===
engine = pyttsx3.init()

def speak(text):
    print("🔈", text)
    engine.say(text)
    engine.runAndWait()

# === Setup: Speech-to-text ===
def get_voice_input():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    with mic as source:
        print("🎙️ Listening...")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    try:
        command = recognizer.recognize_google(audio)
        print(f"🗣️ You said: {command}")
        return command
    except sr.UnknownValueError:
        speak("Sorry, I couldn't understand you.")
        return ""
    except sr.RequestError:
        speak("Speech recognition service is down.")
        return ""

# === LLaMA assistant that can both chat and act ===
def get_assistant_response(user_input):
    # If input includes "start" and "experiment" — switch to execution mode
    trigger_keywords = ["start the experiment", "run the experiment", "begin the protocol", "initiate"]

    if any(kw in user_input.lower() for kw in trigger_keywords):
        system_prompt = (
            "You are AUTOHIAM, a lab robot controller. When the user asks to start the experiment, "
            "you must confirm and include ONLY the keyword 'COMMAND:START'. "
            "DO NOT ask any follow-up questions. Keep the response short and actionable."
        )
    else:
        system_prompt = (
            "You are AUTOHIAM, a friendly lab assistant who chats naturally. "
            "Do not include any command unless the user clearly asks to start the experiment."
        )

    try:
        response = ollama.chat(
            model="llama3",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_input}
            ]
        )
        reply = response['message']['content'].strip()
        print(f"🤖 AUTOHIAM: {reply}")
        return reply
    except Exception as e:
        print(f"❌ Error with LLaMA: {e}")
        return "I'm having trouble responding right now."


# === Main Loop ===
def main():
    while True:
        input_text = get_voice_input()
        if not input_text:
            continue

        response = get_assistant_response(input_text)
        speak(response)

        # Action trigger check
        if "COMMAND:START" in response:
            speak("Starting the experiment now.")
            subprocess.run(["python3", "D:\\PythonProjects\\autohiam\\experiment.py"])

# === Run AUTOHIAM ===
if __name__ == "__main__":
    main()
