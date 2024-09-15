import speech_recognition as sr
from deep_translator import GoogleTranslator

def speech_to_text():
    # 初始化识别器
    recognizer = sr.Recognizer()

    # 使用默认的麦克风
    with sr.Microphone() as source:
        print("Please speak something...")
        audio = recognizer.listen(source)

        # 尝试使用不同的语言进行识别
        languages = ['en-US', 'zh-CN']  # 英语、简体中文
        text_result=""
        for lang in languages:
            try:
                # 使用 Google 的语音识别服务
                text = recognizer.recognize_google(audio, language=lang)
                print(f"Detected ({lang}): " + text)
                text_result=text
                if lang != 'en-US':
                    # 翻译成英语
                    text = GoogleTranslator(source='auto', target='en').translate(text)
                    print("Translated to English: " + text)
                    text_result=text
            except sr.UnknownValueError:
                print(f"Google Speech Recognition could not understand audio in {lang}")
            except sr.RequestError as e:
                print(f"Could not request results from Google Speech Recognition service in {lang}; {e}")
    return text_result
        


text=speech_to_text()
print("The final result is: ",text)