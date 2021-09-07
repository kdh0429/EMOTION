import random 

class Decision:
    """
    BASIC RULE
    1. Default-1
    2. Speaking-5
    3. Depending on Content 2-4
    
    FACE TYPES: 1-Basic / 2-Grin / 3-Oh / 4-Negative / 5-isSpeaking
    """ 
    def __init__(self):
        self.load_keywods()
        self.decision = 1

    def load_keywods(self):
        basic_keywords_array = []
        with open('C:/Users/Dyros/Desktop/avatar_ws/src/EMOTION/keyword/basic_keywords.txt', 'r') as filehandle:
            for line in filehandle:
                currentPlace = line[:-1]
                basic_keywords_array.append(currentPlace)
        self.basic_keywords = set(basic_keywords_array)

        grin_keywords_array = []
        with open('C:/Users/Dyros/Desktop/avatar_ws/src/EMOTION/keyword/grin_keywords.txt', 'r') as filehandle:
            for line in filehandle:
                currentPlace = line[:-1]
                grin_keywords_array.append(currentPlace)
        self.grin_keywords = set(grin_keywords_array)

        wow_keywords_array = []
        with open('C:/Users/Dyros/Desktop/avatar_ws/src/EMOTION/keyword/wow_keywords.txt', 'r') as filehandle:
            for line in filehandle:
                currentPlace = line[:-1]
                wow_keywords_array.append(currentPlace)
        self.wow_keywords = set(wow_keywords_array)

        negative_keywords_array = []
        with open('C:/Users/Dyros/Desktop/avatar_ws/src/EMOTION/keyword/negative_keywords.txt', 'r') as filehandle:
            for line in filehandle:
                currentPlace = line[:-1]
                negative_keywords_array.append(currentPlace)
        self.negative_keywords = set(negative_keywords_array)

    def decide(self, isSpeaking=False, last_sentence=""):
        if isSpeaking:
            self.decision = 5
        else:
            # TODO: Category Algorithm
            category_score = [0,0,0,0]
            sentence = last_sentence.lower().split()
            
            category_score[0] = len(set(sentence) & self.basic_keywords)
            category_score[1] = len(set(sentence) & self.grin_keywords)
            category_score[2] = len(set(sentence) & self.wow_keywords)
            category_score[3] = len(set(sentence) & self.negative_keywords)
                                    
            self.decision = category_score.index(max(category_score)) + 1
            
        return self.decision