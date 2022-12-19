a=5
b=3
for i in range(3):
    with open('obstacle.txt', 'a') as f:
        f.write(f'\n Obstacle position:{a,b}')