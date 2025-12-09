#!/usr/bin/env python3
"""
Execution Logger per simulazioni EXCITE

Gestisce il salvataggio automatico di:
- Parametri di simulazione
- Risultati telemetrici
- Metriche di performance
- Report testuali

Ogni esecuzione viene salvata in executions/YYYYMMDD_HHMMSS_<description>/
"""

import os
import json
import time
from datetime import datetime
from pathlib import Path
import numpy as np


class ExecutionLogger:
    """Logger per tracciare esecuzioni di simulazioni"""

    def __init__(self, base_dir="executions", description="simulation"):
        """
        Inizializza logger

        Args:
            base_dir: Directory base per salvataggio (default: executions/)
            description: Descrizione breve della simulazione
        """
        self.base_dir = Path(base_dir)
        self.description = description.replace(" ", "_")

        # Crea timestamp unico
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_id = f"{self.timestamp}_{self.description}"

        # Crea directory di esecuzione
        self.run_dir = self.base_dir / self.run_id
        self.run_dir.mkdir(parents=True, exist_ok=True)

        # Sottodirectory
        self.telemetry_dir = self.run_dir / "telemetry"
        self.plots_dir = self.run_dir / "plots"
        self.logs_dir = self.run_dir / "logs"

        for dir_path in [self.telemetry_dir, self.plots_dir, self.logs_dir]:
            dir_path.mkdir(exist_ok=True)

        # Metadata della simulazione
        self.metadata = {
            'run_id': self.run_id,
            'timestamp': self.timestamp,
            'description': description,
            'start_time': time.time(),
            'status': 'running'
        }

        print(f"[LOGGER] Execution directory: {self.run_dir}")

    def log_config(self, config_dict):
        """
        Salva configurazione simulazione

        Args:
            config_dict: Dizionario con parametri di configurazione
        """
        config_file = self.run_dir / "config.json"

        with open(config_file, 'w') as f:
            json.dump(config_dict, f, indent=2, default=str)

        print(f"[LOGGER] Configuration saved: {config_file}")

    def log_telemetry(self, data_dict):
        """
        Salva dati telemetrici in formato NumPy

        Args:
            data_dict: Dict con {nome: numpy_array}
        """
        for name, data in data_dict.items():
            file_path = self.telemetry_dir / f"{name}.npy"
            np.save(file_path, data)

        print(f"[LOGGER] Telemetry saved: {len(data_dict)} datasets")

    def log_telemetry_csv(self, data_dict):
        """
        Salva dati telemetrici in formato CSV (per analisi esterna)

        Args:
            data_dict: Dict con {nome: numpy_array}
        """
        for name, data in data_dict.items():
            file_path = self.telemetry_dir / f"{name}.csv"

            # Se è 2D, salva con header
            if data.ndim == 2:
                n_cols = data.shape[1]
                header = ','.join([f'col_{i}' for i in range(n_cols)])
                np.savetxt(file_path, data, delimiter=',', header=header, comments='')
            else:
                np.savetxt(file_path, data, delimiter=',')

        print(f"[LOGGER] CSV telemetry saved: {len(data_dict)} files")

    def log_metrics(self, metrics_dict):
        """
        Salva metriche di performance

        Args:
            metrics_dict: Dict con metriche calcolate
        """
        metrics_file = self.run_dir / "metrics.json"

        with open(metrics_file, 'w') as f:
            json.dump(metrics_dict, f, indent=2, default=str)

        print(f"[LOGGER] Metrics saved: {metrics_file}")

    def log_text_report(self, report_text, filename="report.txt"):
        """
        Salva report testuale

        Args:
            report_text: Stringa con report
            filename: Nome file (default: report.txt)
        """
        report_file = self.run_dir / filename

        with open(report_file, 'w') as f:
            f.write(report_text)

        print(f"[LOGGER] Report saved: {report_file}")

    def log_console_output(self, output_text):
        """
        Salva output console

        Args:
            output_text: Output della simulazione
        """
        console_file = self.logs_dir / "console.log"

        with open(console_file, 'w') as f:
            f.write(output_text)

        print(f"[LOGGER] Console output saved: {console_file}")

    def finalize(self, status='completed', error_msg=None):
        """
        Finalizza logging e salva summary

        Args:
            status: 'completed', 'failed', 'partial'
            error_msg: Messaggio di errore (se status='failed')
        """
        self.metadata['end_time'] = time.time()
        self.metadata['duration_seconds'] = self.metadata['end_time'] - self.metadata['start_time']
        self.metadata['status'] = status

        if error_msg:
            self.metadata['error'] = error_msg

        # Salva metadata finale
        metadata_file = self.run_dir / "metadata.json"
        with open(metadata_file, 'w') as f:
            json.dump(self.metadata, f, indent=2, default=str)

        # Crea summary testuale
        summary = self._generate_summary()
        self.log_text_report(summary, "SUMMARY.txt")

        print(f"[LOGGER] Execution finalized: {status}")
        print(f"[LOGGER] Duration: {self.metadata['duration_seconds']:.1f} seconds")
        print(f"[LOGGER] Results in: {self.run_dir}")

    def _generate_summary(self):
        """Genera summary testuale"""

        duration_str = f"{self.metadata['duration_seconds']:.1f}s"
        if self.metadata['duration_seconds'] > 3600:
            duration_str = f"{self.metadata['duration_seconds']/3600:.2f}h"
        elif self.metadata['duration_seconds'] > 60:
            duration_str = f"{self.metadata['duration_seconds']/60:.1f}min"

        summary = f"""
EXCITE Simulation Execution Summary
{'='*70}

Run ID:       {self.run_id}
Timestamp:    {self.timestamp}
Description:  {self.metadata['description']}
Status:       {self.metadata['status'].upper()}
Duration:     {duration_str}

Directory:    {self.run_dir}

Files Generated:
- config.json         : Simulation parameters
- metadata.json       : Execution metadata
- metrics.json        : Performance metrics
- telemetry/*.npy     : Raw telemetry data
- telemetry/*.csv     : CSV telemetry (for external tools)
- logs/console.log    : Console output
- SUMMARY.txt         : This file

{'='*70}
Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
"""
        return summary

    def get_run_dir(self):
        """Ritorna path directory di esecuzione"""
        return str(self.run_dir)


def create_execution_index(base_dir="executions"):
    """
    Crea/aggiorna indice di tutte le esecuzioni

    Args:
        base_dir: Directory base executions
    """
    base_path = Path(base_dir)

    if not base_path.exists():
        return

    # Trova tutte le esecuzioni
    executions = []
    for run_dir in sorted(base_path.iterdir(), reverse=True):
        if not run_dir.is_dir():
            continue

        metadata_file = run_dir / "metadata.json"
        if metadata_file.exists():
            with open(metadata_file, 'r') as f:
                metadata = json.load(f)
                executions.append({
                    'run_id': metadata.get('run_id', run_dir.name),
                    'timestamp': metadata.get('timestamp', ''),
                    'description': metadata.get('description', ''),
                    'status': metadata.get('status', 'unknown'),
                    'duration_s': metadata.get('duration_seconds', 0)
                })

    # Salva indice
    index_file = base_path / "INDEX.json"
    with open(index_file, 'w') as f:
        json.dump({
            'total_executions': len(executions),
            'last_updated': datetime.now().isoformat(),
            'executions': executions
        }, f, indent=2)

    # Crea README
    readme_text = f"""# EXCITE Simulation Executions

Total executions: {len(executions)}
Last updated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

## Recent Executions

"""
    for i, exec_data in enumerate(executions[:10], 1):
        duration_str = f"{exec_data['duration_s']:.1f}s"
        if exec_data['duration_s'] > 3600:
            duration_str = f"{exec_data['duration_s']/3600:.2f}h"

        status_icon = "✅" if exec_data['status'] == 'completed' else "⚠️"
        readme_text += f"{i}. {status_icon} `{exec_data['run_id']}` - {exec_data['description']} ({duration_str})\n"

    readme_text += f"""
## Directory Structure

Each execution directory contains:
- `config.json` - Simulation configuration parameters
- `metadata.json` - Execution metadata (timestamps, status, duration)
- `metrics.json` - Performance metrics (attitude error, RW speeds, power, etc.)
- `telemetry/` - Raw telemetry data (NumPy .npy and .csv formats)
- `plots/` - Generated plots (if enabled)
- `logs/` - Console output and error logs
- `SUMMARY.txt` - Human-readable summary

## Usage

To analyze a specific execution:
```python
import numpy as np
import json

# Load metrics
with open('executions/<run_id>/metrics.json') as f:
    metrics = json.load(f)

# Load telemetry
sigma_BR = np.load('executions/<run_id>/telemetry/sigma_BR.npy')
```
"""

    readme_file = base_path / "README.md"
    with open(readme_file, 'w') as f:
        f.write(readme_text)

    print(f"[INDEX] Created execution index: {index_file}")
    print(f"[INDEX] Total executions tracked: {len(executions)}")
