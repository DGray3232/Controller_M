#ifndef HPF_H_
#define HPF_H_

// Структура для хранения состояния фильтра
typedef struct {
    float alpha;    // Коэффициент фильтрации
    float prev_x;   // Предыдущее входное значение
    float prev_y;   // Предыдущее выходное значение
} HPF;

// инициализация ФВЧ
void HPF_Init(HPF *filter, float alpha, float initial_value);
// Вычисление отфильтрованного значения
float HPF_Update(HPF *filter, float new_x);

#endif /* HPF_H_ */
