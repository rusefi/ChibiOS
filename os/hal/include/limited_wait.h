#pragma once

// this hook allows rusEFI to fail gracefully instead of on hanging
#ifndef LIMITED_WHILE_LOOP
#define LIMITED_WHILE_LOOP(msg, condition) { (void)msg ; while (condition) ; }
#endif
