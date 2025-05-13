#!/usr/bin/env python3

import unicodedata
from vocab import Vocab
import os

def _is_punctuation(char):
    """Checks whether `chars` is a punctuation character."""
    cp = ord(char)
    # We treat all non-letter/number ASCII as punctuation.
    # Characters such as "^", "$", and "`" are not in the Unicode
    # Punctuation class but we treat them as punctuation anyways, for
    # consistency.
    if ((cp >= 33 and cp <= 47) or (cp >= 58 and cp <= 64) or
        (cp >= 91 and cp <= 96) or (cp >= 123 and cp <= 126)):
        return True
    cat = unicodedata.category(char)
    if cat.startswith("P"):
        return True
    return False

def _run_split_on_punc(text):
    """
    Splits punctuation on a piece of text.
    """
    chars = list(text)
    i = 0
    start_new_word = True
    output = []
    while i < len(chars):
        char = chars[i]
        if _is_punctuation(char):
            output.append([char])
            start_new_word = True
        else:
            if start_new_word:
                output.append([])
            start_new_word = False
            output[-1].append(char)
        i += 1

    return ["".join(x) for x in output]

def _is_control(char):
    """Checks whether `chars` is a control character."""
    # These are technically control characters but we count them as whitespace
    # characters.
    if char == "\t" or char == "\n" or char == "\r":
        return False
    cat = unicodedata.category(char)
    if cat.startswith("C"):
        return True
    return False

def _is_whitespace(char):
    """
    Checks whether `chars` is a whitespace character.
    """
    # \t, \n, and \r are technically contorl characters but we treat them
    # as whitespace since they are generally considered as such.
    if char == " " or char == "\t" or char == "\n" or char == "\r":
        return True
    cat = unicodedata.category(char)
    if cat == "Zs":
        return True
    return False

def _clean_text(text):
    """
    Performs invalid character removal and whitespace cleanup on text.
    """
    output = []
    for char in text:
        cp = ord(char)
        if cp == 0 or cp == 0xfffd or _is_control(char):
            continue
        if _is_whitespace(char):
            output.append(" ")
        else:
            output.append(char)
    return "".join(output)

def _is_chinese_char(cp):
    """
    Checks whether CP is the codepoint of a CJK character.
    """

    # This defines a "chinese character" as anything in the CJK Unicode block:
    #     https://en.wikipedia.org/wiki/CJK_Unified_Ideographs_(Unicode_block)
    #
    # Note that the CJK Unicode block is NOT all Japanese and Korean characters,
    # despite its name. The modern Korean Hangul alphabet is a different block,
    # as is Japanese Hiragana and Katakana. Those alphabets are used to write
    # space-separated words, so they are not treated specially and handled
    # like the all of the other languages.
    if ((cp >= 0x4E00 and cp <= 0x9FFF) or  #
        (cp >= 0x3400 and cp <= 0x4DBF) or  #
        (cp >= 0x20000 and cp <= 0x2A6DF) or  #
        (cp >= 0x2A700 and cp <= 0x2B73F) or  #
        (cp >= 0x2B740 and cp <= 0x2B81F) or  #
        (cp >= 0x2B820 and cp <= 0x2CEAF) or
        (cp >= 0xF900 and cp <= 0xFAFF) or  #
        (cp >= 0x2F800 and cp <= 0x2FA1F)):  #
        return True

    return False

def _tokenize_chinese_chars(text):
    """
    Adds whitespace around any CJK character.
    """
    output = []
    for char in text:
        cp = ord(char)
        if _is_chinese_char(cp):
            output.append(" ")
            output.append(char)
            output.append(" ")
        else:
            output.append(char)
    return "".join(output)

def _run_strip_accents(text):
    """
    Strips accents from a piece of text.
    """
    text = unicodedata.normalize("NFD", text)
    output = []
    for char in text:
        cat = unicodedata.category(char)
        if cat == "Mn":
            continue
        output.append(char)
    return "".join(output)

class BertTokenizer():
    def __init__(self,
                 vocab_file,
                 do_lower_case=True,
                 unk_token="[UNK]",
                 sep_token="[SEP]",
                 pad_token="[PAD]",
                 cls_token="[CLS]",
                 mask_token="[MASK]"):
        if not os.path.isfile(vocab_file):
            raise ValueError(
                "Can't find a vocabulary file at path '{}'. To load the "
                "vocabulary from a pretrained model please use "
                "`tokenizer = BertTokenizer.from_pretrained(PRETRAINED_MODEL_NAME)`"
                .format(vocab_file))
        self.vocab = Vocab.load_vocabulary(vocab_file, unk_token=unk_token)
        self.do_lower_case = do_lower_case

    def convert_to_unicode(self, text):
        if isinstance(text, str):
            return text
        elif isinstance(text, bytes):
            return text.decode("utf-8", "ignore")
        else:
            raise ValueError("Unsupported string type: %s" % (type(text)))

    def whitespace_tokenize(self, text):
        text = text.strip()
        if not text:
            return []
        tokens = text.split()
        return tokens

    def wordpiece_tokenize(self, text):
        output_tokens = []
        for token in self.whitespace_tokenize(text):
            chars = list(token)
            if len(chars) > 100:
                output_tokens.append("[UNK]")
                continue

            is_bad = False
            start = 0
            sub_tokens = []
            while start < len(chars):
                end = len(chars)
                cur_substr = None
                while start < end:
                    substr = "".join(chars[start:end])
                    if start > 0:
                        substr = "##" + substr
                    if substr in self.vocab:
                        cur_substr = substr
                        break
                    end -= 1
                if cur_substr is None:
                    is_bad = True
                    break
                sub_tokens.append(cur_substr)
                start = end

            if is_bad:
                output_tokens.append("[UNK]")
            else:
                output_tokens.extend(sub_tokens)
        return output_tokens

    def basic_tokenize(self, text):
        text = self.convert_to_unicode(text)
        text = _clean_text(text)
        text = _tokenize_chinese_chars(text)

        orig_tokens = self.whitespace_tokenize(text)
        split_tokens = []
        for token in orig_tokens:
            if self.do_lower_case:
                token = token.lower()
                token = _run_strip_accents(token)
            split_tokens.extend(_run_split_on_punc(token))

        output_tokens = self.whitespace_tokenize(" ".join(split_tokens))
        return output_tokens

    def tokenize(self, text):
        split_tokens = []
        for token in self.basic_tokenize(text):
            for sub_token in self.wordpiece_tokenize(token):
                split_tokens.append(sub_token)
        return split_tokens

    def convert_tokens_to_ids(self, tokens):
        """
        Converts a sequence of tokens into ids using the vocab. The tokenizer
        should has the `vocab` attribute.
        Args：
            tokens (list(str)): List of tokens.
        Returns:
            list: Converted id list.
        """
        return self.vocab.to_indices(tokens)

if __name__ == "__main__":
    berttokenizer = BertTokenizer(vocab_file="/home/lemon/robot_ros_application/catkin_ws/src/ros_speech_emotion_recognition/configs/vocab.txt", do_lower_case=True)
    token = berttokenizer.tokenize("He was a puppeteer")
    print(token)
