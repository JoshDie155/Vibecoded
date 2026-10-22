# PartJoin

Mehrere 3D-Modelle, die eigentlich zusammengehören (z. B. ein in Teile zerlegter Download),
**über markierte Punkte automatisch passgenau zu einem Modell verbinden** – damit man nach dem
3D-Druck nichts mehr zusammensetzen und kleben muss.

Läuft komplett im Browser, keine Installation, keine Daten verlassen den Rechner.

## So funktioniert's

1. **Modelle laden** – Dateien ins Fenster ziehen (STL, OBJ mit MTL, 3MF, PLY, AMF, GLB/GLTF, FBX, DAE, WRL, 3DS, VTK).
   Mehrteilige Dateien werden in einzelne Teile mit ihrer Farbe zerlegt – auch 3MF-Projekte aus
   Bambu Studio, OrcaSlicer und PrusaSlicer (Farbe aus der Filament-Zuordnung). Teile eines Objekts
   bilden eine Gruppe (⛓) und bewegen sich gemeinsam; jedes Teil lässt sich aus seiner Gruppe lösen
   und per Farbwähler umfärben. Liegen die Teile in den Dateien schon richtig zueinander,
   „Lage aus der Datei übernehmen" anhaken.
2. **Automatisch oder von Hand auswählen** – **„Alles automatisch verbinden"** sucht für alle Teile
   passende Flächen (deckungsgleiche Schnittflächen, Zapfen und Bohrungen) und setzt sie ab dem
   Basisteil zusammen; **„Vorschlagen"** macht das für eine einzelne Verbindung, „Nächster Vorschlag"
   probiert weitere. Von Hand: oben im 3D-Fenster zwischen **Punkte** (Taste P) und **Flächen** (Taste F) wechseln.
   - *Flächen:* die Kontaktfläche auf beiden Teilen anklicken. PartJoin erkennt selbst, ob es eine
     ebene Fläche, eine Kreisfläche oder ein Zylinder (Zapfen bzw. Bohrung) ist, und setzt die Teile
     wie in einem CAD-Programm aufeinander: Ebene auf Ebene, Kreis mittig auf Kreis, Zapfen koaxial
     in Bohrung. Mehrere Flächenpaare (z. B. Auflagefläche + Zapfen) legen die Lage vollständig fest.
   - *Punkte:* auf beiden Teilen dieselben markanten Stellen anklicken (Ecken, Kanten …).
     **Die Reihenfolge ist egal** – PartJoin erkennt selbst, welche Markierung zu welcher gehört.
   Punkte und Flächen lassen sich auch kombinieren.
3. **Verbinden** – das bewegliche Teil wird exakt an das feste angesetzt. Was die Auswahl offen lässt
   (z. B. die Drehung um einen Zapfen), sucht *Automatisch einpassen* über Flächenkontakt ohne
   Durchdringung: Teile mit Abflachung, Nut oder Lochbild drehen sich so von selbst richtig, und ein
   Zapfen wird bis zum Anschlag eingeschoben.
   *Von Hand anpassen* blendet ein Gizmo ein – standardmäßig nur für die Richtungen, die noch frei
   sind (z. B. nur der Drehring um die Zapfenachse), mit Winkelraster 1°/5°/15°/45°/90°.
   *Feinausrichtung* zieht die Kontaktflächen genau aufeinander, *Einsinken* schiebt die Teile minimal
   ineinander, damit der Slicer sie sicher verschmilzt.
4. **Prüfen** – „Passung prüfen" färbt ein, wo sich die Teile berühren (grün), wo ein Spalt bleibt
   (gelb, z. B. das Spiel eines Zapfens) und wo sie sich durchdringen (rot), mit Maßen je Teilepaar.
5. **Exportieren**
   - **3MF mehrfarbig**: ein Objekt aus mehreren Teilen, jedes mit eigenem Namen, eigener Farbe und
     eigenem Extruder je Farbe – im Slicer jedem Extruder ein Filament zuordnen.
   - **STL, OBJ, 3MF (ein Körper)**: mit „Zu einem Körper vereinigen" werden die Teile per
     boolescher Vereinigung zu einem geschlossenen Netz (Überlappungen und Innenflächen verschwinden).
     Nicht geschlossene Teile werden unverändert angehängt und genannt.

Mehr als zwei Teile: einfach weitere Verbindungen anlegen. Alles wird ausgehend vom **Basisteil**
zusammengesetzt (Basis in der Teileliste wählbar).

Bedienung: Linksklick wählt aus (nochmal klicken hebt eine Fläche wieder auf) · Rechtsklick auf eine
Markierung oder Fläche löscht sie · Strg+Z nimmt die letzte Auswahl zurück · P/F wechselt Punkte/Flächen ·
im Gizmo G = verschieben, R = drehen, Esc = fertig · Ziehen dreht die Ansicht, Mausrad zoomt.
Jedes Teil lässt sich über ✥ in der Teileliste auch frei von Hand bewegen.

## Runde Teile

Bei Kreisen und Zylindern legt die Form die Drehung um die Achse nicht fest. PartJoin geht so vor:

1. Achsen und Mittelpunkte werden exakt ausgerichtet (Zapfen ↔ Bohrung koaxial, Kreisfläche ↔ Kreisfläche
   mittig). Das Spiel zwischen Zapfen und Bohrung wird angezeigt.
2. Die Drehung wird über den ganzen Kreis abgetastet und die Stellung mit dem besten Flächenkontakt ohne
   Durchdringung gewählt. Hat das Teil irgendetwas Unrundes (Abflachung, Nase, Lochbild, Kanten), ist die
   Stellung damit eindeutig.
3. Ist das Teil wirklich rund, sagt PartJoin das – dann ist die Drehung frei: mit dem Gizmo (Drehring um die
   Achse, Raster z. B. 15°) einstellen oder einen zusätzlichen Punkt auf beiden Teilen setzen.

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
- **Flächenerkennung**: vom angeklickten Dreieck aus wird die Fläche über Nachbardreiecke erweitert und als
  Ebene (mit Umriss und ggf. Kreis), Zylinder (Achse, Radius, Zapfen/Bohrung, Bogenwinkel) oder Freiform
  erkannt (`src/math/features.ts`).
- **Paarungen**: Flächenpaare werden zu Bedingungen (Ebene auf Ebene, koaxial, Punkt auf Punkt); ein
  Levenberg-Marquardt-Löser mit mehreren Startlagen findet die Lage und bestimmt, welche Bewegungen danach
  noch frei sind (`src/math/mate.ts`, `src/math/constraints.ts`).
- **Automatisch einpassen**: freie Drehungen/Verschiebungen werden abgetastet und nach Kontakt minus
  Durchdringung bewertet (`src/math/scan.ts`), danach nur entlang freier Richtungen genau angelegt
  (`src/math/settle.ts`); gleiche Umrisse werden deckungsgleich gelegt (`src/math/outline.ts`).
- **Vorschläge**: markante Flächen je Teil (`src/math/segment.ts`), Paare über drehungsunabhängige
  Formmerkmale (Fläche, Umfang, Trägheitsradien) bzw. Durchmesser, verifiziert durch Aufeinanderlegen
  und Durchdringungsprüfung (`src/math/suggest.ts`).
- **Prüfansicht**: Abstand und Vorzeichen zur Oberfläche des anderen Teils je (unterteiltem) Dreieck
  (`src/math/check.ts`).
- **3MF** lesen/schreiben inkl. Slicer-Metadaten (`src/io/threemf.ts`), **Vereinigung** mit
  manifold-3d (`src/io/union.ts`).

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
- Projekt speichern/laden
- Druckausrichtung (größte ebene Fläche aufs Druckbett)
- Passungshilfe (Zapfen gezielt dünner machen)
- Bemalte Bereiche (Bambu „paint_color") beim 3MF-Import als eigene Teile
