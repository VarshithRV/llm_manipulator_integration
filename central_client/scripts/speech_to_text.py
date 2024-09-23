import speech_recognition as sr

recognizer = sr.Recognizer()

def list_microphones():
    # List available microphones
    print("Available Microphones:")
    for index, name in enumerate(sr.Microphone.list_microphone_names()):
        print(f"Microphone {index}: {name}")
    return sr.Microphone.list_microphone_names()

def speech_to_text():
    mic_list = list_microphones()
    if not mic_list:
        print("No microphones found. Please check your audio setup.")
        return

    with sr.Microphone() as source:
        print("Adjusting for ambient noise, please wait...")
        recognizer.adjust_for_ambient_noise(source)
        print("Listening for speech...")
        audio = recognizer.listen(source)

        try:
            print("Recognizing speech...")
            text = recognizer.recognize_google(audio)
            print("You said: " + text)
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand the audio.")
        except sr.RequestError as e:
            print(f"Could not request results from Google Speech Recognition; {e}")

if __name__ == "__main__":
    speech_to_text()