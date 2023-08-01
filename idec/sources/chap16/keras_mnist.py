import matplotlib.pyplot as plt
import tensorflow as tf

mnist = tf.keras.datasets.mnist

# 훈련 데이터와 테스트 데이터를 가져온다.
(x_train, y_train),(x_test, y_test) = mnist.load_data()	


# 넘파이를 사용하여 입력을 0.0에서 1.0 사이로 만든다. 
x_train, x_test = x_train / 255.0, x_test / 255.0
plt.imshow(x_train[0], cmap="Greys")
model = tf.keras.models.Sequential()

model.add(tf.keras.layers.Flatten(input_shape=(28,28)))
model.add(tf.keras.layers.Dense(512, activation='relu'))
model.add(tf.keras.layers.Dropout(0.2))
model.add(tf.keras.layers.Dense(10, activation='softmax'))
model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])
model.fit(x_train, y_train, epochs=5)
model.evaluate(x_test, y_test)