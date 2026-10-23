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
   probiert weitere. Von Hand: oben im 3D-Fenster zwischen **Flächen** (Taste F, Standard), **Kanten** (Taste K),
   **Punkte** (Taste P) und **Pinsel** (Taste B) wechseln.
   - *Flächen:* die Kontaktfläche auf beiden Teilen anklicken. PartJoin erkennt selbst, ob es eine
     ebene Fläche, eine Kreisfläche oder ein Zylinder (Zapfen bzw. Bohrung) ist, und setzt die Teile
     wie in einem CAD-Programm aufeinander: Ebene auf Ebene, Kreis mittig auf Kreis, Zapfen koaxial
     in Bohrung. Mehrere Flächenpaare (z. B. Auflagefläche + Zapfen) legen die Lage vollständig fest.
   - *Punkte:* auf beiden Teilen dieselben markanten Stellen anklicken (Ecken, Kanten …).
     **Die Reihenfolge ist egal** – PartJoin erkennt selbst, welche Markierung zu welcher gehört.
   - *Kanten:* gerade Kante auf gerade Kante (z. B. bündig an einer Außenkante) – gut kombinierbar mit einer Fläche.
   - *Pinsel:* mit gedrückter Maustaste über die Kontaktstelle streichen; beim Loslassen wird daraus eine
     Fläche (eben oder Freiform). Praktisch, wo die automatische Flächenerkennung nicht passt.
   - *Flächen zusammenfassen:* in der Flächenliste zwei oder mehr Flächen anhaken und „… zu einer
     zusammenfassen" klicken – oder mit Alt/⌥ + Klick eine Fläche direkt an die zuletzt gewählte anhängen.
     Das geht auch über fest verbundene Teile hinweg (z. B. ein Aussparungsgrund, der halb zu den Haaren und
     halb zum Kopf gehört); „⇹" trennt sie wieder.
   Punkte, Kanten und Flächen lassen sich auch kombinieren. Besteht das Gegenstück aus mehreren Teilen (z. B.
   eine Maske, die auf Kopf *und* Haare passt), dürfen Flächen auch auf Nachbarteilen liegen, die schon fest mit
   Teil 1 verbunden sind – PartJoin rechnet und prüft den Kontakt dann mit allen diesen Teilen.
3. **Verbinden** – das bewegliche Teil wird exakt an das feste angesetzt. Was die Auswahl offen lässt
   (z. B. die Drehung um einen Zapfen), sucht *Automatisch einpassen* über Flächenkontakt ohne
   Durchdringung: Teile mit Abflachung, Nut oder Lochbild drehen sich so von selbst richtig, und ein
   Zapfen wird bis zum Anschlag eingeschoben. Zwei etwa gleich große ebene Flächen (Schnittflächen,
   Zapfen-Stirn auf Aussparung) kommen mittig aufeinander; gesucht wird dann nur noch die Drehung.
   Passt eine andere Drehung genauso gut (z. B. 180° bei rechteckigen Zapfen), wechselt
   „Um 180° drehen" mit einem Klick.
   Kommt zu einer fertigen Verbindung eine weitere Fläche oder ein Punkt hinzu, bleibt das Teil an
   seiner Stelle, bis auch das Gegenstück gewählt ist.
   *Von Hand anpassen* blendet ein Gizmo ein (volle Drehringe, Pfeile zum Verschieben). Standardmäßig
   lassen sich nur die Richtungen greifen, die noch frei sind (z. B. nur der Drehring um die
   Zapfenachse) – gesperrte Achsen erscheinen als graue gestrichelte Linie. Winkelraster 1°/5°/15°/45°/90°.
   *Feinausrichtung* zieht die Kontaktflächen genau aufeinander, *Einsinken* schiebt die Teile minimal
   ineinander, damit der Slicer sie sicher verschmilzt.
4. **Prüfen** – „Passung prüfen" färbt ein, wo sich die Teile berühren (grün), wo ein Spalt bleibt
   (gelb, z. B. das Spiel eines Zapfens) und wo sie sich durchdringen (rot), mit Maßen je Teilepaar.
5. **Exportieren**
   - **3MF für Bambu Studio / OrcaSlicer** (Projektformat von Bambu): ein Objekt aus mehreren Teilen
     mit eigenem Namen; jede Teilefarbe wird ein Filament in genau dieser Farbe. Die Datei bringt
     Projekteinstellungen für den gewählten Drucker mit (System-Profile von Bambu Studio: 0,4-mm-Düse,
     0,20 mm Standard, Bambu PLA Basic; auch H2C, H2D, H2D Pro und X2D mit zwei Düsen) – ohne sie lädt
     Bambu Studio ab 2.8.2 nur die Geometrie.
     Der Drucker eines geladenen Bambu-Projekts wird vorausgewählt.
   - **3MF für PrusaSlicer**: ein Objekt aus mehreren Teilen; Teile gleicher Farbe bekommen dieselbe
     Extrudernummer – im Slicer jeder Nummer das passende Filament zuordnen.
   - **STL, OBJ, 3MF (ein Körper)**: mit „Zu einem Körper vereinigen" werden die Teile per
     boolescher Vereinigung zu einem geschlossenen Netz (Überlappungen und Innenflächen verschwinden).
     Nicht geschlossene Teile werden unverändert angehängt und genannt.

Mehr als zwei Teile: einfach weitere Verbindungen anlegen. Alles wird ausgehend vom **Basisteil**
zusammengesetzt (Basis in der Teileliste wählbar).

Bedienung: Linksklick wählt aus (nochmal klicken hebt eine Fläche wieder auf) · Rechtsklick auf eine
Markierung oder Fläche löscht sie · Strg+Z nimmt die letzte Auswahl zurück · P/F wechselt Punkte/Flächen ·
im Gizmo G = verschieben, R = drehen, Esc = fertig · linke Maustaste dreht die Ansicht, mittlere oder
rechte verschiebt sie, Mausrad zoomt zum Mauszeiger · Strg/⌘ + Ziehen schiebt ein Teil (mit allem, was
daran hängt) über die Bauplatte. Nach „Verbinden" bleibt die Ansicht, wie sie ist.
Namen: Mauszeiger kurz über einem Teil halten zeigt seinen Namen, Shift+Klick zeigt es in der Teileliste,
Klick auf einen Namen lässt das Teil aufleuchten, Doppelklick benennt es um. Die Seitenleiste lässt sich
am Rand breiter oder schmaler ziehen (Doppelklick: zurücksetzen).
„Alle Teile" über der Teileliste blendet alle Teile aus oder wieder ein, der Pfeil davor klappt die
Liste ein. Ausgeblendete Teile stehen in einem eigenen, einklappbaren Abschnitt; die Auswahllisten der
Verbindungen zeigen nur sichtbare Teile.
Jedes Teil lässt sich über das Pfeilkreuz in der Teileliste frei von Hand bewegen, „Gesamtes Modell bewegen" verschiebt
oder dreht das ganze zusammengesetzte Modell.

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
