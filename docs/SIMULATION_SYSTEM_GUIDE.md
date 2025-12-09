# EXCITE Simulation System - Guida Completa

**Data creazione**: 2025-12-09
**Status**: ✅ Sistema pronto per test estesi

---

## Overview

Sistema completo di simulazione e testing per il satellite EXCITE con logging automatico, analisi performance e tracciamento esecuzioni.

### Stato Testing

| Test | Durata | Status | Data |
|------|--------|--------|------|
| Test rapido | 5 secondi | ✅ PASS | 2025-12-09 |
| Test breve | 1 minuto | ✅ PASS | 2025-12-09 |
| Test medio | 10 minuti | ✅ PASS | 2025-12-09 |
| Test lungo | 1 ora | ✅ PASS | 2025-12-09 |
| **Test completo** | **24 ore** | ⏳ PENDING | - |

### Risultati Performance (Test 1 ora)

- **Errore assetto**: 0.000° (ECCELLENTE)
- **Velocità angolare**: 0.0000°/s (ECCELLENTE)
- **RW utilization**: 0.09% (OTTIMO - enorme margine disponibile)
- **Power budget**: Stabile

---

## Script Disponibili

### 1. `test_sim.py` - Quick Validation
**Scopo**: Test rapido di 5 secondi per verificare che la simulazione si avvii correttamente.

```bash
source venv_alberto/bin/activate
python test_sim.py
```

**Output**: Messaggio di successo/errore
**Quando usarlo**: Dopo modifiche al codice, per quick check

---

### 2. `test_incremental.py` - Test di Durata
**Scopo**: Test di durata variabile senza logging persistente.

**Uso**:
```bash
source venv_alberto/bin/activate
python test_incremental.py
```

**Configurazione**: Modificare manualmente la durata nel file (linea 28):
```python
scenario.ConfigureStopTime(macros.sec2nano(3600.0))  # 3600s = 1 ora
```

**Output**: Solo console, nessun salvataggio
**Quando usarlo**: Per test veloci durante sviluppo

---

### 3. `tests/test_mission_phases.py` - Analisi Dettagliata
**Scopo**: Analisi approfondita con calcolo metriche performance.

**Uso**:
```bash
source venv_alberto/bin/activate
python tests/test_mission_phases.py --duration 1.0
python tests/test_mission_phases.py --duration 24.0
```

**Output**:
- Report dettagliato a console
- Metriche calcolate (attitude, omega, RW, power)
- Rating performance (EXCELLENT/GOOD/WARNING)

**Quando usarlo**: Per validare performance dopo modifiche controllo

---

### 4. `run_tracked_simulation.py` - **Sistema Completo con Logging** ⭐
**Scopo**: Esecuzione completa con salvataggio automatico in `executions/`.

**Uso**:
```bash
source venv_alberto/bin/activate

# Test breve (1 minuto)
python run_tracked_simulation.py --duration 0.0167 --description "test_breve"

# Test 1 ora
python run_tracked_simulation.py --duration 1.0 --description "test_1h"

# Test completo 24 ore
python run_tracked_simulation.py --duration 24.0 --description "mission_completa"
```

**Output salvato in** `executions/YYYYMMDD_HHMMSS_<description>/`:
```
executions/20251209_013350_logging_test/
├── config.json              # Configurazione simulazione
├── metadata.json            # Timestamp, durata, status
├── metrics.json             # Metriche performance (JSON)
├── REPORT.md                # Report markdown dettagliato
├── SUMMARY.txt              # Summary testuale
├── telemetry/
│   ├── sigma_BR.npy         # Telemetria raw (NumPy)
│   ├── sigma_BR.csv         # Telemetria CSV
│   ├── omega_BR_B.npy
│   ├── omega_BR_B.csv
│   ├── rw_speeds_rad_s.npy
│   ├── rw_speeds_rad_s.csv
│   ├── battery_level_wh.npy
│   └── ... (9 dataset totali)
├── logs/
│   └── console.log          # Output console completo
└── plots/                   # (vuoto - future implementazioni)
```

**Indice Automatico**: `executions/INDEX.json` e `executions/README.md` aggiornati automaticamente

**Quando usarlo**:
- Test formali da documentare
- Run da condividere con il team
- Simulazioni lunghe (>1 ora)
- Quando serve tracciare risultati nel tempo

---

### 5. `scripts/run_simulation.py` - Entry Point Standard
**Scopo**: Entry point principale del progetto (legacy, usare `run_tracked_simulation.py` invece).

**Uso**:
```bash
python scripts/run_simulation.py --duration 24.0 --plots
```

---

## Workflow Raccomandato

### Durante Sviluppo
1. **Quick check**: `python test_sim.py` (5s)
2. **Test modifiche**: `python test_incremental.py` (1-10 min)
3. **Validazione**: `python tests/test_mission_phases.py --duration 1.0`

### Test Formali
1. **Run tracciato**:
   ```bash
   python run_tracked_simulation.py --duration 1.0 --description "after_control_tuning"
   ```
2. **Revisione risultati**: Aprire `executions/<run_id>/REPORT.md`
3. **Confronto**: Comparare `metrics.json` tra diverse run
4. **Analisi approfondita**: Caricare telemetria da `.npy` o `.csv`

### Test Completo Missione (24 ore)
```bash
# Avvia simulazione tracciata
python run_tracked_simulation.py --duration 24.0 --description "mission_24h_baseline"

# Risultati in:
# executions/YYYYMMDD_HHMMSS_mission_24h_baseline/
```

---

## Analisi Dati Telemetrici

### Python (NumPy)
```python
import numpy as np
import matplotlib.pyplot as plt

# Carica telemetria
run_id = "20251209_013350_logging_test"
sigma_BR = np.load(f'executions/{run_id}/telemetry/sigma_BR.npy')
time = np.load(f'executions/{run_id}/telemetry/time.npy')

# Plot errore assetto
plt.plot(time, np.linalg.norm(sigma_BR, axis=1))
plt.xlabel('Time [hours]')
plt.ylabel('Attitude Error [MRP magnitude]')
plt.show()
```

### CSV (Excel, MATLAB, etc.)
Tutti i dati sono disponibili anche in formato CSV in `telemetry/*.csv` per analisi esterna.

---

## Metriche Calcolate

### 1. Attitude Error (MRP)
- **Mean error**: Errore medio durante simulazione
- **Max error**: Picco massimo di errore
- **Final error**: Errore a fine simulazione
- **Target**: < 1° (EXCELLENT), < 5° (GOOD)

### 2. Angular Velocity
- **Mean**: Velocità angolare media
- **Max**: Picco velocità
- **Final**: Velocità finale
- **Target**: < 0.01°/s (EXCELLENT), < 0.1°/s (GOOD)

### 3. Reaction Wheels
- **Max speeds**: Velocità massima per ogni RW (RPM)
- **Mean speeds**: Velocità media per ogni RW
- **Peak overall**: Massimo tra tutte le RW
- **Saturation**: % utilizzo rispetto a limite 6000 RPM
- **Target**: < 80% (GOOD), < 95% (WARNING), ≥95% (CRITICAL)

### 4. Power Budget
- **Initial/Final SOC**: State of Charge batteria (%)
- **Min SOC**: SOC minimo raggiunto
- **Delta SOC**: Variazione netta
- **Target**: Min SOC > 30%

---

## Gestione Executions

### Pulizia Automatica
Le directory in `executions/` crescono nel tempo. Per pulizia:

```bash
# Mantieni solo le ultime 10 esecuzioni
cd executions
ls -t | tail -n +11 | xargs rm -rf

# Oppure cancella run fallite
find . -name "metadata.json" -exec grep -l '"status": "failed"' {} \; | xargs dirname | xargs rm -rf
```

### Backup Esecuzioni Importanti
```bash
# Copia run importante fuori da executions/
cp -r executions/20251209_120000_mission_24h_baseline ~/backup/excite_runs/
```

---

## Troubleshooting

### Problema: Simulazione lenta
**Soluzione**: Verificare che non ci siano altri processi pesanti attivi. La simulazione 24h può richiedere diverse ore di calcolo.

### Problema: File telemetria troppo grandi
**Soluzione**: Per run molto lunghe (>24h), considerare:
- Aumentare sampling time in scenario.py
- Salvare solo metriche aggregate invece di raw telemetry

### Problema: Errore import moduli
**Soluzione**: Verificare virtual environment attivo:
```bash
source venv_alberto/bin/activate
python -c "import Basilisk; print(Basilisk.__version__)"
```

---

## Prossimi Passi

### Immediate
1. ✅ Sistema di logging completo - FATTO
2. ⏳ Test completo 24 ore
3. ⏳ Validazione transizioni FSM

### Future Improvements
- [ ] Integrazione plotting automatico in `run_tracked_simulation.py`
- [ ] Dashboard web per visualizzazione executions/
- [ ] Comparazione automatica tra run diverse
- [ ] Alert automatici per anomalie (RW saturation, low SOC, etc.)
- [ ] Export PDF report automatico

---

## File di Configurazione Chiave

| File | Contenuto | Quando Modificare |
|------|-----------|-------------------|
| `excite/config/spacecraft.py` | Massa, inerzia, geometria | Cambio hardware satellite |
| `excite/config/actuators.py` | RW, MTB, thruster specs | Cambio attuatori |
| `excite/config/sensors.py` | Star Tracker, IMU, TAM | Cambio sensori |
| `excite/config/control.py` | Guadagni controllori | Tuning controllo |
| `excite/config/mission.py` | Orbita, timeline, IC | Cambio missione |

---

## Riferimenti

- **Report testing**: `TESTING_REPORT.md`
- **Documentazione Basilisk**: http://hanspeterschaub.info/basilisk
- **Architettura completa**: `docs/0_context.md`
- **Piano refactoring**: `docs/refactoring_plan.md`

---

**Documento creato**: 2025-12-09
**Ultima revisione**: 2025-12-09
**Status**: ✅ Sistema pronto per test estesi
