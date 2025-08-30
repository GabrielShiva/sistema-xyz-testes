#include "globals.h"

// Inicializa o array que salva as posições da letra especificada
void init_saved_positions(void) {
    for (int i = 0; i < MAX_SAVED_POSITIONS; i++) {
        saved_positions[i].character = '\0';
        saved_positions[i].x_position = 0;
        saved_positions[i].y_position = 0;
        saved_positions[i].is_used = false;
    }
}

// Envia as posições que estão salvas no array para a GUI
void send_saved_positions_update(void) {
    printf("SAVED_POSITIONS");

    for (int i = 0; i < MAX_SAVED_POSITIONS; i++) {
        if (saved_positions[i].is_used) {
            printf(",%c,%d,%d",
                   saved_positions[i].character,
                   saved_positions[i].x_position,
                   saved_positions[i].y_position);
        }
    }

    printf("\n");
}

// Encontra o item do array que contém a letra especificada via interface (serial)
int find_saved_position_index(char character) {
    for (int i = 0; i < MAX_SAVED_POSITIONS; i++) {
        if (saved_positions[i].is_used && saved_positions[i].character == character) {
            return i;
        }
    }

    // Caso a letra não seja encontrada, retorna -1
    return -1;
}