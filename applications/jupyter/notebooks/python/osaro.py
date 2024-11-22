
import numpy
import pandas
import json

dataframe = pandas.read_csv('osaro_imdb_json.csv')

actor_lists = []
for actor_list in dataframe['actors_list']:
    a_list = json.loads(actor_list)
    actor_lists.append(a_list)


def search(actor1, actor2, separation):

    min_sep = -1

    for a_list in actor_lists:
        if len(a_list) == 0:
            pass

        elif actor1 in a_list and actor2 in a_list:
            return separation

        elif actor1 in a_list and len(a_list) > 1:
            a_list.remove(actor1)

            for actor in a_list:
                sep = search(actor, actor2, separation + 1)
                if sep < min_sep or min_sep == -1:
                    min_sep = sep

    return min_sep


def solution(actor1, actor2):
    sep = search(actor1, actor2, 0)
    return sep

print("answer: ", solution("Ralph Fiennes", "Ben Kingsley"))

