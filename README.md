# PartJoin

Mehrere 3D-Modelle, die eigentlich zusammengehören (z. B. ein in Teile zerlegter Download),
**über markierte Punkte automatisch passgenau zu einem Modell verbinden** – damit man nach dem
3D-Druck nichts mehr zusammensetzen und kleben muss.

Läuft komplett im Browser, keine Installation, keine Daten verlassen den Rechner.

## So funktioniert's

1. **Modelle laden** – Dateien ins Fenster ziehen (STL, OBJ, 3MF, PLY, AMF, GLB/GLTF, FBX, DAE, WRL, 3DS, VTK).
   Ab zwei Teilen wird automatisch eine Verbindung angelegt.
2. **Punkte markieren** – auf beiden Teilen dieselben markanten Stellen anklicken (Ecken, Kanten,
   Zapfen, Löcher …). Mit „An Ecken einrasten" springt der Punkt auf die nächste Ecke des Netzes.
   **Die Reihenfolge ist egal**: PartJoin probiert alle Zuordnungen durch und erkennt selbst,
   welche Markierung zu welcher gehört.
3. **Verbinden** – das bewegliche Teil wird exakt an das feste angesetzt.
   Optional: *Feinausrichtung* zieht die Kontaktflächen rund um die Markierungen genau aufeinander,
   *Einsinken* schiebt die Teile minimal ineinander, damit der Slicer sie sicher verschmilzt.
4. **Exportieren** – als STL, OBJ oder 3MF (ein Objekt, oder positionierte Einzelteile z. B. für Mehrfarbdruck).

Mehr als zwei Teile: einfach weitere Verbindungen anlegen. Alles wird ausgehend vom **Basisteil**
zusammengesetzt (Basis in der Teileliste wählbar).

Bedienung: Linksklick setzt eine Markierung · Rechtsklick auf eine Markierung löscht sie ·
Strg+Z nimmt die letzte zurück · Ziehen dreht die Ansicht, Mausrad zoomt.

## Wie viele Punkte?

| Punkte | Ergebnis |
| --- | --- |
| 1 | Teile berühren sich am Punkt, Flächen liegen aufeinander. Drehung um die Kontaktachse per Regler. |
| 2 | Lage eindeutig, wenn die Flächennormalen helfen (Standard). |
| 3+ | Am genauesten. Bei symmetrischen Anordnungen (z. B. Quadrat) warnt PartJoin, wenn mehrere Lösungen unterschiedlich aussehen – „Alternative" schaltet durch. |

## Wie es intern funktioniert

- **Starre Ausrichtung**: Horn-Verfahren (Quaternionen) – findet Rotation + Verschiebung mit der
  kleinsten Abweichung zwischen den Punktpaaren, Spiegelungen ausgeschlossen (`src/math/align.ts`).
- **Automatische Zuordnung**: alle Permutationen der Markierungen werden bewertet (bei vielen Punkten
  mit Abstandsprüfung als Vorfilter, da Abstände bei starren Bewegungen gleich bleiben).
  Flächennormalen gehen als Hilfspunkte ein: an einer Kontaktstelle zeigen die Oberflächen
  gegeneinander (`src/math/match.ts`).
- **Symmetrie-Erkennung**: Lösungen, die das Teil geometrisch auf dieselbe Form abbilden
  (z. B. ein um 180° gedrehtes rechteckiges Dach), gelten nicht als Mehrdeutigkeit.
- **Feinausrichtung**: ICP auf Oberflächenstichproben nahe der Markierungen; nur Flächen, die
  gegeneinander zeigen, zählen als Kontakt (`src/math/icp.ts`, BVH via `three-mesh-bvh`).

## Entwicklung

```bash
npm install
npm run dev            # Entwicklungsserver
npm test               # Unit-Tests der Mathematik
npm run build          # Build nach dist/
npm run build:single   # eine einzige, eigenständige HTML-Datei: dist-single/index.html
```

## Ideen für später

- CAD-Formate STEP/IGES (z. B. über `occt-import-js`)
- Echte boolesche Vereinigung zu einem wasserdichten Netz (z. B. `manifold-3d`)
- Automatische Erkennung passender Bruch-/Kontaktflächen ganz ohne Markierungen
- Projekt speichern/laden
