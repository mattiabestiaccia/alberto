# Contesto del Progetto Basilisk

## Architettura Generale

Il progetto si basa su **Basilisk**, che utilizza un'architettura ibrida:

- **Core**: Funzioni scritte in C e/o C++ per le performance.
- **Scripting**: Interfaccia e configurazione gestite tramite script Python.
- **Moduli Custom**: Oltre ai moduli nativi, il progetto utilizza moduli C/C++ sviluppati ad hoc (es. QUEST e SMEKF). Se non trovi riferimenti nella documentazione ufficiale, è per questo motivo.

## Struttura dei File Python

Il progetto è organizzato in 4 script principali:

### 1. `EXCITE_Dynamics.py`

Gestisce la fisica e l'ambiente della simulazione:

- Dinamica dello spacecraft, attuatori e sensori.
- Ambiente e attrattori (Terra, Luna, Sole).
- Disturbi ambientali (Pressione solare, disturbi magnetici, drag atmosferica, gravity gradient).

### 2. `EXCITE_Fsw.py` (Flight Software)

Simula il software che girerebbe sull'On-Board Computer (OBC):

- Contiene gli algoritmi di **Guida, Navigazione e Controllo (GNC)**.
- Gestisce la logica della macchina a stati della missione (detumbling, deployment pannelli, puntamento Sole/Ground Station, ecc.).
- **Nota**: È qui che risiedono gli algoritmi **QUEST** e **SMEKF** oggetto del lavoro attuale.

### 3. `EXCITE_scenario.py`

È l'orchestratore della simulazione:

- Mette in comunicazione _Dynamics_ e _Fsw_.
- Gestisce l'esecuzione dello scenario.
- Contiene varie helper functions.

### 4. `EXCITE_Plotting.py`

Libreria di utility per il plotting dei risultati delle simulazioni.

---

## Focus Attuale: Algoritmi di Navigazione (in `EXCITE_Fsw.py`)

L'obiettivo è far funzionare in tandem due algoritmi di navigazione.

### 1. QUEST (Attitude Determination)

- **Descrizione**: Algoritmo che risolve il "problema di Wahba". Date due misure vettoriali in terna _Body_ e le corrispondenti in terna _Inerziale_, calcola la matrice di rotazione (assetto) tra le due terne.
- **Input utilizzati**:
  - _Terna Body_: Vettore direzione Sole (da `simpleNavObj` in Dynamics, usato al posto dei sun sensors) + Output del Magnetometro.
  - _Terna Inerziale_: Direzione Sole + Intensità campo magnetico.
- **Stato Attuale**: **Non funzionante**. Ci sono problemi nel far convergere o funzionare correttamente l'algoritmo con gli input attuali.

### 2. SMEKF (Sequential Multiplicative Extended Kalman Filter)

- **Descrizione**: Filtro di navigazione d'assetto (Kalman Filter).
- **Funzionamento**: Fonde le misure di:
  - Giroscopio
  - Star Tracker
  - Output del QUEST
- **Stato Attuale**: **Disabilitato** nella simulazione per permettere il debug del QUEST.
- **Strategia**: Per il debug, è consigliabile riattivarlo escludendo l'input del QUEST e utilizzando solo lo **Star Tracker** (che è affidabile) per verificare il funzionamento base del filtro.
