import numpy as np
import pandas as pd
import requests
from io import StringIO
from sklearn.tree import DecisionTreeClassifier
from sklearn.utils import shuffle
from sklearn import metrics
from time import sleep


def build_model_from_url(url):
    df = download_file_from_url(url)
    train, test = split_data(df)
    return build_model(DecisionTreeClassifier(), train, test)


def download_file_from_url(url, retries=3):
    if retries <= 0:
        print("Unable to download model!!!!!!!!!!!!!!!!")
        return
    try:
        response = requests.get(url)
        if response.status_code == 200:
            data = StringIO(response.text)
            df = pd.read_csv(data, sep=',', names=["Age", "Workout", "Vaccine"])
            return df
        print("Error downloading data")
    except Exception as err:
        print(f"Connection error: {err}")
        sleep(2)
        download_file_from_url(url, retries - 1)


def split_data(df):
    df = shuffle(df)
    msk = np.random.rand(len(df)) < 0.7
    train, test = df[msk], df[~msk]
    return train, test


def build_model(model, train, test):
    model.fit(train[['Age', 'Workout']], train[['Vaccine']])
    # predict = model.predict(test[['Age', 'Workout']])
    # name = 'RandomForest'
    # results = printResults(name, test[['Vaccine']], predict, model, False)
    return model


def printResults(name, y_test, y_pred, model, featureImportance=False):
    ca = str(round(metrics.accuracy_score(y_test, y_pred),4))
    pre = str(round(metrics.precision_score(y_test, y_pred, average='macro'),4))
    rec = str(round(metrics.recall_score(y_test, y_pred, average='macro'),4))

    print("---\nModel: ", name)
    if featureImportance:
        print("Feature importance: ", model.feature_importances_)
    print("Classification accuracy: ", ca)
    print("Percision: ", pre)
    print("Recall: ", rec)

    return [ca, pre, rec]

