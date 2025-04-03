#!/usr/bin/env python3
import subprocess
import time

def activate_vscode_window():
    time.sleep(1)

    try:
        # Suche nach Fenstern, die "Visual Studio Code" im Titel haben
        window_ids = subprocess.check_output(['xdotool', 'search', '--name', 'Visual Studio Code']).decode('utf-8').strip().split()

        if window_ids:
            # Aktiviere das erste gefundene Fenster
            subprocess.check_call(['xdotool', 'windowactivate', window_ids[0]])
            print(f"VSCode-Fenster mit ID {window_ids[0]} aktiviert.")
        else:
            print("Kein VSCode-Fenster gefunden.")

    except subprocess.CalledProcessError as e:
        print("Fehler beim Aktivieren des VSCode-Fensters: ", e)

if __name__ == "__main__":
    activate_vscode_window()
