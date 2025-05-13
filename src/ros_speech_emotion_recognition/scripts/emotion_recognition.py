#!/usr/bin/env python3

import onnxruntime as rt
from bert_tokenizer import BertTokenizer
import numpy as np
from custom_constant import Constants
Constants.SYS_MODULE.path.append(Constants.OS_MODULE.path.join(Constants.ROSPKG_MODULE.RosPack().get_path("leju_lib_pkg"), "scripts"))
import url_downloading

DOWNLOAD_SUCCEED = True

class EmotionRecognition(BertTokenizer):
    def __init__(self, vocab_path, do_lower_case=True):
        super().__init__(vocab_path, do_lower_case)
        self.load_model()

    def load_model(self):
        model_path = Constants.EMOTION_RECOGNITION_MODEL
        if Constants.OS_MODULE.path.exists(model_path):
            if url_downloading.md5sum(model_path) == Constants.EMOTION_RECOGNITION_MODEL_MD5:
                try:
                    self.model = rt.InferenceSession(model_path)
                    return
                except Exception as e:
                    print("加载 {} 模型失败!\n{}".format(Constants.EMOTION_RECOGNITION_MODEL_FILE_NAME, e))
                    exit(1)
            else:
                Constants.OS_MODULE.remove(model_path)
        download_result = url_downloading.download_from_url(
            Constants.MODEL_DOWNLOAD_URL,
            Constants.EMOTION_RECOGNITION_MODEL_FOLDER,
            Constants.EMOTION_RECOGNITION_MODEL_FILE_NAME,
            Constants.EMOTION_RECOGNITION_MODEL_MD5
        )
        if download_result == DOWNLOAD_SUCCEED:
            try:
                self.model = rt.InferenceSession(model_path)
            except Exception as e:
                print("加载 {} 模型失败!\n{}".format(Constants.EMOTION_RECOGNITION_MODEL_FILE_NAME, e))
                exit(1)
        else:
            exit("下载 {} 模型失败！".format(Constants.EMOTION_RECOGNITION_MODEL_FILE_NAME))

    def convert_featrue(self, sample, max_len):
        tokens = self.tokenize(sample)
        if len(tokens) > max_len - 2:
            tokens = tokens[:max_len - 2]
        tokens = ["[CLS]"] + tokens + ["[SEP]"]
        input_ids = self.convert_tokens_to_ids(tokens)
        attention_mask = [1] * len(input_ids)
        assert len(input_ids) == len(attention_mask)
        if len(input_ids) < max_len:
            input_ids = input_ids + [0] * (max_len - len(input_ids))
            attention_mask = attention_mask + [0] * (max_len - len(attention_mask))
        return input_ids, attention_mask

    def batch_data(self, text_list, max_len):
        input_ids_list, attention_mask_list = [], []
        for single_text in text_list:
            input_ids, attention_mask = self.convert_featrue(single_text, max_len)
            input_ids_list.append(input_ids)
            attention_mask_list.append(attention_mask)
        return {"input_ids": np.array(input_ids_list, dtype=np.int64),
                "attention_mask": np.array(attention_mask_list, dtype=np.int64)}

    def run(self, text):
        batch = self.batch_data(text, Constants.MAX_LEN)
        input_ids = batch["input_ids"]
        attention_mask = batch["attention_mask"]
        model_inputs = {self.model.get_inputs()[0].name: input_ids, self.model.get_inputs()[1].name: attention_mask}
        pred_label = self.model.run(None, model_inputs)[0]
        label_name = [Constants.ID2LABEL[pred] for pred in pred_label]
        return label_name

if __name__ == '__main__':
    er = EmotionRecognition(Constants.VOCAB_FILE_PATH)
    text = ["今天天气真差，但是我很开心"]
    emotion = er.run(text)
    print(emotion)
