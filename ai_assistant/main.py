import speech_recognition as sr
import pyttsx3
import subprocess


def speak(text):
    engine = pyttsx3.init()
    engine.say(text)
    engine.runAndWait()


def listen_and_respond():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    with mic as source:
        print("Listening...")
        audio = recognizer.listen(source)

    try:
        command = recognizer.recognize_google(audio).lower()
        print(f"You said: {command}")
        if "hello" in command and "start" in command and "experiment" in command:
            speak("Yes, starting the experiment.")
            subprocess.run(["python3", "/path/to/your/experiment_script.py"])
        else:
            speak("Sorry, I didn't understand.")
    except sr.UnknownValueError:
        speak("I couldn't understand what you said.")
    except sr.RequestError:
        speak("Speech service is down.")


listen_and_respond()
