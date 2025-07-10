import pyttsx3

engine = pyttsx3.init()          # already in your code
voices = engine.getProperty("voices")

for v in voices:
    print("ID:", v.id)
    try:
        langs = v.languages[0].decode()
    except:
        langs = "Unknown"
    print("Languages:", langs)
    print("Name:", v.name)
    print("---")
