import random 

# TODO: Word Bag System
basic_keywords = {'hm', 'hmm', 'well'}
grin_keywords = {'hello', 'goodbye', 'thanks', 'thank', 'good', 'understand', 'ha', 'funny', 'yeah', 'yay'}
wow_keywords = {'wow', 'oh', 'ho', 'ooh', 'amazing', 'beautiful', 'Hooray'}
negative_keywords={'strange', 'awkward','hate', 'weird', 'mistake', 'sorry'}

class Decision:
    """
    BASIC RULE
    1. Default-1
    2. Speaking-5
    3. Depending on Content 2-4
    
    FACE TYPES: 1-Basic / 2-Grin / 3-Oh / 4-Negative / 5-isSpeaking
    """ 
    def __init__(self):
        self.decision = 1

    def decide(self, isSpeaking=False, last_sentence=""):
        if isSpeaking:
            self.decision = 5
        else:
            # TODO: Category Algorithm
            category_score = [0,0,0,0]
            sentence = last_sentence.lower().split()
            
            category_score[0] = len(set(sentence) & basic_keywords)
            category_score[1] = len(set(sentence) & grin_keywords)
            category_score[2] = len(set(sentence) & wow_keywords)
            category_score[3] = len(set(sentence) & negative_keywords)
                                    
            self.decision = category_score.index(max(category_score)) + 1
            
        return self.decision