# Assegnazione pin finale (SAM C21 – baseline G / superset J)

## Infrastruttura
| Funzione | Pin | Note |
|--------|-----|------|
| VREFA | PA03 | riferimento analogico |
| XTAL 24 MHz | PA14 (XIN), PA15 (XOUT) | quarzo esterno obbligatorio |
| SWD | PA30, PA31 | debug |
| RESET | RESETN | — |

## Analog
| Funzione | Pin | Note |
|--------|-----|------|
| ADC (unico ingresso) | PA02 | unico AIN usato |
| AC interno | PA12 | CMP, pin distinto dall’ADC |
| Freq rete (digitale) | PA27 | da comparatore esterno → EIC/EVSYS |

## Timer / CCL (core logica)
| Funzione | Pin | Note |
|--------|-----|------|
| TC one-shot OUT | PB12 | WO attivo durante il conteggio |
| CCL LUT0 OUT (must) | PB02 | DGI SPI ignorata |
| CCL LUT1 OUT (must) | PA11 | uscita logica principale |
| CCL LUT2 OUT (debug) | PB09 | uscita DFF (opzionale) |
| LUT3 | interno | parte del DFF |

## TCC (heartbeat + debug)
| Funzione | Pin | Note |
|--------|-----|------|
| TCC WO[0] | PA08 | debug timing |
| TCC WO[1] | PA09 | debug timing |
| TCC WO[2] | PA10 | debug timing |
| TCC interno | — | heartbeat di sistema |

## Comunicazioni

### UART
| Tipo | Pin | Note |
|-----|-----|------|
| UART primaria (VCOM) | PB10 (TX), PB11 (RX) | UART standard via EDBG |
| UART applicativa | PA22, PA23 | opzionale |

### SPI
| Tipo | Pin | Note |
|-----|-----|------|
| SPI slave applicativa | PA17 (SS), PA18 (MOSI), PA16 (MISO), PA19 (SCK) | SERCOM1 |
| SPI EDBG (DGI) | PB00, PB01, PB02 | non usata, pin liberi |

## GPIO applicativi
| Funzione | Pin |
|--------|-----|
| Switch analogici (4) | PA20, PA21, PB04, PB05 |
| Trigger IN | PB14 |
| Trigger OUT | PB15 |
| LED error | esterno (pin libero sul PCB) |

## Note operative
- Baseline G (48-pin), identica su J (64-pin).
- Compatibile con ATSAMC21 Xplained Pro senza modifiche.
- Led della scheda condiviso con il quarzo (ponticellare/rimuovere)
