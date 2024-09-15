import speech_recognition as sr
from deep_translator import GoogleTranslator
from gtts import gTTS
import pygame
import io

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
        
# text=speech_to_text()
# print("The final result is: ",text)

def text_to_speech(text, lang='en'):
    """将文本转换为语音并直接播放"""
    tts = gTTS(text=text, lang=lang, slow=False)  # 创建gTTS对象
    mp3_fp = io.BytesIO()
    tts.write_to_fp(mp3_fp)  # 写入内存文件
    mp3_fp.seek(0)           # 移动到文件的开始

    pygame.mixer.init()      # 初始化pygame混音器
    pygame.mixer.music.load(mp3_fp)  # 加载内存中的音频文件
    pygame.mixer.music.play()        # 播放音频
    while pygame.mixer.music.get_busy():  # 检查音乐流是否正在播放
        pygame.time.Clock().tick(10)  # 等待播放结束



text = "Hello, how are you today? \n I'm apple"  # 要转换的文本
text_to_speech(text, 'en')