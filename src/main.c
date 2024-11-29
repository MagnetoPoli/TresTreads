#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#define MY_STACK_SIZE 1024  // Definindo o tamanho da pilha das threads
#define MY_PRIORITY 0       // Definindo a prioridade das threads

// Definições dos nós dos pinos do botão e LEDs
#define SW0_NODE  DT_ALIAS(sw0)
#define LED0 DT_ALIAS(led0)
#define LED1 DT_ALIAS(led1)
#define LED2 DT_ALIAS(led2)

// Definindo mutexes para sincronização das threads
K_MUTEX_DEFINE(mutex1);
K_MUTEX_DEFINE(mutex2);

// Especificações do botão e LEDs
static const struct gpio_dt_spec botao = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback botao_cb_data;
static const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET_OR(LED0, gpios, {0}),
    GPIO_DT_SPEC_GET_OR(LED1, gpios, {0}),
    GPIO_DT_SPEC_GET_OR(LED2, gpios, {0}),
};

volatile int botao_state = 1;  // Estado do botão

// Callback para quando o botão for pressionado
void botao_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    printk("botao apertado\n");
    botao_state = !botao_state;  // Alternando o estado do botão
}

// Função para alternar o estado do LED
void muda_led1(const struct gpio_dt_spec *led, struct k_mutex *mutex, int delay) {
    gpio_pin_configure_dt(led, GPIO_OUTPUT_ACTIVE);  // Configura o LED como saída
    while (1) {
        if (k_mutex_lock(mutex, K_FOREVER) == 0) {
            gpio_pin_toggle_dt(led);  // Alterna o estado do LED
            k_msleep(delay);  // Espera pelo tempo definido no delay
            k_mutex_unlock(mutex);  // Libera o mutex
            k_msleep(delay);  // Espera pelo tempo definido no delay
        }
    }
}

// Função para controlar o LED baseado no estado do botão
void led_ativo(void) {
    if (!gpio_is_ready_dt(&botao)) {
        printk("botao não funcionando\n");
        return;
    }

    gpio_pin_configure_dt(&botao, GPIO_INPUT | GPIO_INT_EDGE_BOTH);  // Configura o botão como entrada com interrupção
    gpio_pin_interrupt_configure_dt(&botao, GPIO_INT_EDGE_BOTH);  // Configura a interrupção para ambas as bordas
    gpio_init_callback(&botao_cb_data, botao_pressed, BIT(botao.pin));  // Inicializa o callback do botão
    gpio_add_callback(botao.port, &botao_cb_data);  // Adiciona o callback do botão

    for (int i = 0; i < ARRAY_SIZE(leds); ++i) {
        if (!gpio_is_ready_dt(&leds[i])) {
            printk("LED não está pronto\n");
            return;
        }
        gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_ACTIVE);  // Configura os LEDs como saída
    }

    printk("Press the botao\n");
    while (1) {
        if (botao_state) {
            if (k_mutex_lock(&mutex2, K_FOREVER) == 0) {
                gpio_pin_set_dt(&leds[0], 1);  // Liga o LED
            }
        } else {
            k_mutex_unlock(&mutex2);
            gpio_pin_set_dt(&leds[0], 0);  // Desliga o LED
        }
        k_msleep(500);  // Espera por 500ms
    }
}

// Função para controlar a interação entre as threads e LEDs
void threads_ativos(void) {
    gpio_pin_configure_dt(&leds[2], GPIO_OUTPUT_ACTIVE);  // Configura o LED2 como saída
    while (1) {
        if (k_mutex_lock(&mutex1, K_FOREVER) == 0) {
            if (k_mutex_lock(&mutex2, K_MSEC(100)) == 0) {
                gpio_pin_set_dt(&leds[2], 1);  // Liga o LED2
                k_msleep(200);  // Espera por 200ms
                k_mutex_unlock(&mutex2);  // Libera o mutex2
                k_mutex_unlock(&mutex1);  // Libera o mutex1
            } else {
                k_mutex_unlock(&mutex1);  // Libera o mutex1 caso mutex2 não esteja disponível
            }
        } else {
            printk("mutex1 travado\n");
        }
        gpio_pin_set_dt(&leds[2], 0);  // Desliga o LED2
        k_msleep(1000);  // Espera por 1000ms
    }
}

// Definição das threads
K_THREAD_DEFINE(led_thread, MY_STACK_SIZE, (k_thread_entry_t)muda_led1, &leds[1], &mutex1, (void*)250, MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(botao_thread, MY_STACK_SIZE, (k_thread_entry_t)led_ativo, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(complex_thread, MY_STACK_SIZE, (k_thread_entry_t)threads_ativos, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
