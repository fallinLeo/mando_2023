

from collections import deque
from collections import Counter
max_queue_size = 13
values_queue = deque(maxlen=max_queue_size)
initial_values = [0,0,0,0,0,0,0,0,0,0,0,0,0]
values_queue.extend(initial_values)


#큐에서 가장 많이 등장한 값 추출
def most_common_value(values_queue):
    value_counts = Counter(values_queue)
    most_common_value = value_counts.most_common(1)[0][0]
    #most_common_count = value_counts.most_common(1)[0][1]
    return most_common_value   

def most_common_count(values_queue):
    value_counts = Counter(values_queue)
    most_common_count = value_counts.most_common(1)[0][1]
    return most_common_count
print(most_common_value(values_queue))
for i in range(1,13):
    values_queue.append(i)
    print(most_common_value(values_queue))
    i+=1

