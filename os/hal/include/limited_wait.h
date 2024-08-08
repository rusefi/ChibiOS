#pragma once

// this hook allows rusEFI to fail gracefully instead of on hanging
#ifndef LIMITED_WHILE_LOOP
#define LIMITED_WHILE_LOOP(condition, ...) { while (condition) ; }
#endif
