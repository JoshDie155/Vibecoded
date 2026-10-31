# D3DForge

*(früher „PartJoin“)*

Eine Werkstatt im Browser für 3D-Druckteile. Im Kern: Mehrere Modelle, die eigentlich zusammengehören (z. B. ein
in Teile zerlegter Download), **über markierte Flächen oder Punkte automatisch passgenau zu einem Modell
verbinden** – damit man nach dem Druck nichts mehr zusammensetzen und kleben muss. Dazu alles rund um den Druck:
zerschneiden und auf Plattengröße zerteilen (mit Zapfen, Dübeln, Schwalbenschwanz oder Schnappstiften),
Passungen anpassen, Aussparungen, Beschriften und Nummerieren, mehrfarbig bemalen, messen, prüfen, für den Druck
ausrichten und auf Platten verteilen, Montageanleitung – und Export für die gängigen Slicer (Bambu Studio,
OrcaSlicer und seine Ableger, PrusaSlicer, Cura) und Drucker vieler Hersteller.

Läuft komplett im Browser, keine Installation, keine Daten verlassen den Rechner.

## So funktioniert's

Im Programm zeigt **?** (Knopf rechts oben oder Taste ?) eine Kurzanleitung mit allen Tastenkürzeln
und Mausaktionen.

**Aufbau**: Oben die Kopfleiste mit den Werkzeugen (Auswahl, Ansehen & Messen, Bearbeiten), Rückgängig,
Wiederholen, Hilfe und „Exportieren“. Links die Seitenleiste mit drei Reitern – **Teile** (laden, Liste,
Beispiele), **Verbinden** (Verbindungen) und **Drucken** (prüfen, ausrichten, Filament & Kosten, Export,
Montageanleitung); am Tablet lässt sich auch zwischen ihnen wischen. Ein offenes Werkzeug (Messen, Malen,
Beschriften, Passung, Zerschneiden …) zeigt seine Einstellungen im Feld rechts im 3D-Fenster.

1. **Modelle laden** – Dateien ins Fenster ziehen (STL, OBJ mit MTL, 3MF, STEP, IGES, BREP, PLY, AMF, GLB/GLTF, FBX,
   DAE, WRL, 3DS, VTK). CAD-Dateien (STEP/IGES/BREP) liest OpenCascade im Hintergrund und trianguliert sie
   fein für den Druck (Sehnenfehler 0,02 mm); jeder Körper wird ein Teil mit Name und Farbe aus der Datei,
   Körper einer Unterbaugruppe bilden eine Gruppe.
   Mehrteilige Dateien werden in einzelne Teile mit ihrer Farbe zerlegt – auch 3MF-Projekte aus
   Bambu Studio, OrcaSlicer und PrusaSlicer (Farbe aus der Filament-Zuordnung). Teile eines Objekts
   bilden eine Gruppe (⛓) und bewegen sich gemeinsam; jedes Teil lässt sich aus seiner Gruppe lösen
   und per Farbwähler umfärben. Liegen die Teile in den Dateien schon richtig zueinander,
   „Lage aus der Datei übernehmen" anhaken.
   **Fester Boden** („Auf die Druckplatte stellen", Standard): Jede Baugruppe – alles, was über
   Verbindungen oder Gruppen zusammenhängt – steht mit ihrem tiefsten Punkt auf der Druckplatte,
   egal welches Teil die Basis ist. So schwebt im Slicer nichts, obwohl der Export ein einziges
   Objekt ist. Teile, die mit „Lage aus der Datei übernehmen" geladen wurden, stehen gemeinsam auf der
   Platte und bleiben untereinander, wie sie in den Dateien liegen (z. B. Kopf über Körper); ebenso die
   Körper einer CAD-Baugruppe und die Netze einer mehrteiligen OBJ-, GLB- oder FBX-Datei. Das Gizmo
   verschiebt Teile dann nur über die Platte; zum freien Anheben den Haken entfernen – alles bleibt
   dabei, wo es steht.
   Aus Slicer-Projekten kommen auch **Aufschriften und Logos als Modifier** (z. B. eingelassener Text
   in anderem Filament), **Farbbemalung**, Fuzzy-Skin-, Stützen- und Naht-Bemalung sowie die
   Druckeinstellungen je Teil mit: Sie werden angezeigt (die Oberfläche erscheint dort in der
   Filamentfarbe, wo sie im Modifier liegt – so wie gedruckt) und beim Export nach Bambu Studio
   wieder mitgeschrieben.
   **Farben malen** (oben „Malen", Taste N) – für mehrfarbigen Druck mit Mehrfarbsystem oder mehreren Düsen,
   wie im Slicer: Oben stehen die Filamente als nummerierte Felder, in derselben Reihenfolge wie im Export
   (die Teilefarben, dann die bemalten); „+" fügt eine übliche Filamentfarbe oder eine eigene hinzu.
   **Pinsel**: mit gedrückter linker Maustaste über das Teil streichen (Größe = Durchmesser, auch mit Alt +
   Mausrad) – große Dreiecke werden am Pinselrand fein unterteilt, gemalt wird nur, was zur Kamera zeigt.
   **Füllen**: eine Fläche anklicken – sie wird bis zu Knicken über dem eingestellten Winkel gefüllt.
   Umschalt radiert (zurück zum Filament des Teils), Ziehen neben dem Teil dreht die Ansicht, jeder Strich
   lässt sich mit Strg+Z zurücknehmen. Die Bemalung kommt ins Projekt, in die Sitzungssicherung und als
   Farbbemalung ins 3MF für Bambu Studio / OrcaSlicer (und PrusaSlicer).
   **Projekt speichern/öffnen**: „Projekt speichern" (Strg/⌘+S) legt alle Teile mit Lage, Farben,
   Gruppen, Aufschriften, Bemalung und allen Verbindungen in einer `.d3dforge`-Datei ab (ältere `.partjoin`-Projekte öffnen weiterhin); „Projekt
   öffnen" oder Hineinziehen macht dort weiter (die vorige Arbeit holt Strg+Z zurück). Zusätzlich
   sichert D3DForge die Sitzung laufend im Browser – nach einem Neuladen bietet es an, den letzten
   Stand wiederherzustellen.
2. **Automatisch oder von Hand auswählen** – **„Alles automatisch verbinden"** sucht für alle Teile
   passende Flächen (deckungsgleiche Schnittflächen, Zapfen und Bohrungen) und setzt sie ab dem
   Basisteil zusammen; **„Vorschlagen"** macht das für eine einzelne Verbindung, „Nächster Vorschlag"
   probiert weitere. Von Hand: oben im 3D-Fenster zwischen **Flächen** (Taste F, Standard), **Kanten** (Taste K),
   **Punkte** (Taste P) und **Polygone** (Taste B) wechseln.
   - *Flächen:* die Kontaktfläche auf beiden Teilen anklicken. D3DForge erkennt selbst, ob es eine
     ebene Fläche, eine Kreisfläche oder ein Zylinder (Zapfen bzw. Bohrung) ist, und setzt die Teile
     wie in einem CAD-Programm aufeinander: Ebene auf Ebene, Kreis mittig auf Kreis, Zapfen koaxial
     in Bohrung. Mehrere Flächenpaare (z. B. Auflagefläche + Zapfen) legen die Lage vollständig fest.
   - *Punkte:* auf beiden Teilen dieselben markanten Stellen anklicken (Ecken, Kanten …).
     **Die Reihenfolge ist egal** – D3DForge erkennt selbst, welche Markierung zu welcher gehört.
   - *Kanten:* gerade Kante auf gerade Kante (z. B. bündig an einer Außenkante) – gut kombinierbar mit einer Fläche.
   - *Polygone:* einzelne Dreiecke markieren – mit gedrückter Maustaste über die Kontaktstelle streichen; beim
     Loslassen wird daraus eine Fläche (eben oder Freiform). Über eine Auswahl streichen hebt sie wieder auf.
     Praktisch, wo die automatische Flächenerkennung nicht passt.
   - *Flächen zusammenfassen:* in der Flächenliste zwei oder mehr Flächen anhaken und „… zu einer
     zusammenfassen" klicken – oder mit Alt/⌥ + Klick eine Fläche direkt an die zuletzt gewählte anhängen.
     Das geht mit ebenen, runden und Freiformflächen (nicht mit Kanten) und auch über fest verbundene Teile
     hinweg (z. B. ein Aussparungsgrund, der halb zu den Haaren und halb zum Kopf gehört). Liegt alles auf
     einem Zylinder (etwa eine Rundung, deren grobe Facette einzeln erkannt wurde), bleibt das Ergebnis
     eine runde Fläche; „⇹" trennt sie wieder.
   Punkte, Kanten und Flächen lassen sich auch kombinieren. Besteht das Gegenstück aus mehreren Teilen (z. B.
   eine Maske, die auf Kopf *und* Haare passt), dürfen Flächen auch auf Nachbarteilen liegen, die schon fest mit
   Teil 1 verbunden sind – D3DForge rechnet und prüft den Kontakt dann mit allen diesen Teilen.
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
   **Messen** (oben „Messen", Taste M) funktioniert wie im Slicer: Unter dem Zeiger leuchtet das
   Merkmal auf – Ecke, gerade Kante, Kreis oder Bogen (Bohrung, runde Außenkante, auch abgerundete Ecken),
   Seite eines Vielecks (Sechskant), ebene Fläche oder auf Rundungen der ganze Zapfen bzw. die Bohrung. Schon
   ein Merkmal zeigt sein Maß: eine Kante ihre Länge, ein Kreis seinen Durchmesser (im Bild und in der
   Leiste), eine Fläche ihren Inhalt. Zwei Merkmale ergeben je nach Art Winkel, (senkrechten) Abstand,
   Achsabstand, direkten Abstand und ΔX/ΔY/ΔZ – jeweils mit Kopierknopf; Kreise zählen dabei mit ihrer Mitte.
   Ein Klick auf den Mittelpunkt eines gewählten Kreises nimmt die Mitte, Umschalt + Klick einen beliebigen
   Punkt; ein weiterer Klick ersetzt das zweite Merkmal, ein Klick auf ein gewähltes wählt es ab. Entf bzw.
   „Neu" beginnt von vorn, Esc nimmt ein Merkmal zurück und beendet ohne Auswahl das Messen.
   **Was gerade an ist**, steht rechts unter den Druckplatten: Werkzeug, Maße, Überhänge & Wände,
   Standfestigkeit, Passungsprüfung, Schnitt, Explosion – mit Farblegende und ✕ zum Ausschalten. Die Leisten
   oben stapeln sich untereinander, wenn mehrere offen sind.
   **Maße** (oben „Maße", Taste D): Breite, Tiefe und Höhe jeder Baugruppe – oder der ausgewählten Teile – als
   Bemaßung im Bild (X rot, Y grün, Z blau), immer an den Kanten zur Kamera; was fast in Blickrichtung liegt
   (z. B. die Höhe in der Draufsicht), fällt weg.
   Die **Statuszeile** unten zeigt Teile, Dreiecke, Verbindungen und die Maße der Baugruppe unter dem Zeiger
   (mit geschätztem Gewicht).
   **Standfestigkeit**: zeigt für jede Baugruppe – so, wie sie gerade steht – den Schwerpunkt (mit Lot zum
   Boden) und die Standfläche (alles, was den Boden berührt) und sagt, ab welcher Neigung sie kippt: grün ab
   15°, gelb knapp, rot wackelig oder kippt um (Schwerpunkt neben der Standfläche). Der Schwerpunkt rechnet
   mit Wänden und Füllung wie bei „Filament & Kosten“ – bei wenig Füllung tragen die Wände das Gewicht.
   **Filament & Kosten**: Material (PLA, PETG, ABS, ASA, TPU, PC, PA, PLA-CF, PETG-CF), Füllung, Wände und
   Preis je kg wählen – D3DForge schätzt Gramm, Meter Filament (1,75 mm) und Kosten aller sichtbaren Teile und
   je Platte. Gerechnet wird wie im Slicer: Wände, Boden und Deckel voll, das Innere mit der Füllung; Teile aus
   Slicer-Projekten mit eigenen Wänden oder eigener Füllung nutzen diese. Ohne Stützen, Brim und Spülturm –
   genau rechnet der Slicer. Die Vorgaben bleiben im Browser gespeichert.
   **Drucker** (Auswahl oben rechts, nach Hersteller gruppiert): Bambu Lab, Prusa, Creality, Anycubic, Elegoo,
   QIDI, Sovol, Voron, Snapmaker, Flashforge, Artillery und UltiMaker – mit Druckbereich und -höhe nach
   Herstellerangabe (bei widersprüchlichen Angaben der kleinere Wert). Der Drucker bestimmt Platte, Prüfung
   „zu groß“, Zerteilen und die Vorauswahl des Slicers beim Export.
   **Druckplatte**: Der Umriss der Platte des gewählten Druckers (mit 10-mm-Raster) liegt im Ursprung;
   rot, wenn eine Baugruppe daneben liegt oder zu groß ist (mit Hinweis in der Statuszeile). „Auf der Platte
   anordnen" (Taste A) legt alle sichtbaren Baugruppen platzsparend darauf; passt nicht alles auf eine Platte,
   füllt es weitere. **Mehrere Platten** liegen wie im Slicer im Raster (etwa √n Spalten, weitere Reihen
   davor). Die Leiste „Platten" unter dem Ansichtswürfel blendet die Platten ein und aus, springt per Nummer
   zu einer Platte, fügt mit + eine hinzu und entfernt mit − die leeren (auch im Rechtsklick-Menü ins Leere).
   Ändert sich die Zahl der Platten oder der Drucker, wandern die Teile mit ihrer Platte. Ein Slicer-Projekt
   (Bambu Studio, OrcaSlicer und Ableger) kommt mit seinen Platten: jedes Objekt auf seine Platte, an
   dieselbe Stelle. Liegt alles auf den Platten, übernimmt der Export ins Projekt genau diese Lage – mit
   derselben Aufteilung auf Platten.
   **Zwei Düsen mit eingeschränkter Reichweite** (z. B. Bambu Lab H2D, H2D Pro, H2C, X2D): Getönte Streifen
   am Plattenrand zeigen, was nur die linke (orange) bzw. nur die rechte Düse (lila) erreicht – Werte aus den
   Druckerprofilen (H2D: links 25 mm, rechts 25 mm; H2C: links 25 mm, rechts 5 mm; X2D: links 20,5 mm). Liegt
   eine Baugruppe darin, sagt es die Statuszeile: Der Slicer druckt dort nur mit den Filamenten dieser Düse.
   Drucker mit Werkzeugwechsler oder unabhängigen Druckköpfen, deren Werkzeuge die ganze Platte erreichen
   (Prusa XL, Snapmaker U1/J1, UltiMaker S-Serie), brauchen das nicht.
   **Druckausrichtung**: „Für den Druck ausrichten" legt jede Baugruppe mit ihrer größten ebenen
   Auflage auf die Druckplatte (bei fast gleicher Fläche die stabilere Lage mit tieferem Schwerpunkt);
   was schon gut liegt, bleibt. Von Hand: oben „Hinlegen" (Taste L), dann die Fläche
   anklicken, die unten liegen soll – die ganze Baugruppe dreht sich mit. Strg+Z macht beides rückgängig.
   **Passung**: oben „Passung" (Taste T), dann einen Zapfen, eine Bohrung oder eine ebene Fläche
   anklicken und das neue Maß eingeben – z. B. Zapfen 0,2 mm dünner (Schnellknöpfe −0,1/−0,2/−0,3),
   Bohrung weiter oder eine Fläche zurücksetzen. Nur die Form ändert sich, die Verbindungen bleiben,
   wo sie sind; so entsteht genau dieser Spalt (gut für getrennt gedruckte Steckteile oder bewegliche
   Gelenke). Bei Bohrungen und Zapfen nimmt D3DForge die ganze Wand – auch wenn ein Schnitt (z. B. aus
   dem Slicer) durch die Bohrung geht und ein Teil der Wand im anderen Teil liegt, oder unsaubere Netze
   sie zerstückeln; der Durchmesser wird aus allen Stücken bestimmt. „auch in …" weitet die Stücke in den
   angrenzenden Teilen mit (abhaken, wenn nur ein Teil mehr Spiel bekommen soll). Strg+Z macht es rückgängig.
5. **Exportieren** (oben „Exportieren“ oder Reiter „Drucken“): in der Liste den Slicer wählen – vorausgewählt
   ist der des gewählten Druckers –, dann „Exportieren“.
   - **Bambu Studio** und **OrcaSlicer & Ableger** (Creality Print, Anycubic Slicer Next, ElegooSlicer,
     QIDI Studio, Snapmaker Orca, Orca-Flashforge – sie lesen alle das Projektformat von Bambu Studio):
     **jede Baugruppe ein eigenes Objekt** aus ihren Teilen (mit eigenem Namen), die Objekte liegen
     zueinander wie in D3DForge und lassen sich im Slicer einzeln anordnen; jede Teilefarbe wird ein
     Filament in genau dieser Farbe. Bei Bambu-Lab-Druckern bringt die Datei die Projekteinstellungen mit
     (System-Profile: 0,4-mm-Düse, 0,20 mm Standard, Bambu PLA Basic; auch die Drucker mit zwei Düsen);
     bei Druckern anderer Hersteller nur Druckbereich und Filamentfarben – Drucker- und Prozessprofil wählt
     man nach dem Öffnen im Slicer. Der Drucker eines geladenen Projekts wird vorausgewählt.
     Modifier (Aufschriften, negative Volumen, Stützen-Volumen), Bemalung und Druckeinstellungen je
     Teil (Wände, Infill, Muster …) bleiben erhalten; Texte aus Bambu Studio bleiben dort bearbeitbar
     (Textangaben, eigenes Netz und Lage werden mitgeschrieben); die Filamentnummern werden dabei auf die
     Filamente des Exports umgestellt. Einstellungen, die nur für ein ganzes Objekt gelten (z. B.
     Schichthöhe, Brim), bleiben erhalten, solange alle Teile eines Objekts dieselben Werte mitbringen;
     sonst gilt das Druckprofil.
   - **PrusaSlicer / SuperSlicer**: ein Objekt aus mehreren Teilen; Teile gleicher Farbe bekommen dieselbe
     Extrudernummer – im Slicer jeder Nummer das passende Filament zuordnen. Die Bemalung kommt mit,
     Modifier nicht (andere Programme würden sie als Teil des Körpers drucken).
   - **UltiMaker Cura**: jedes Teil ein eigenes Objekt mit Namen, Farbe und Extruder-Nummer (1. Farbe =
     Extruder 1 …, Eintrag `cura:extruder_nr`, den Cura beim Öffnen übernimmt), so auf der Platte wie hier.
     Bemalung kennt Cura nicht.
   - **STL, OBJ, 3MF (ein Körper)**: mit „Zu einem Körper vereinigen" werden die Teile per
     boolescher Vereinigung zu einem geschlossenen Netz (Überlappungen und Innenflächen verschwinden).
     Nicht geschlossene Teile werden unverändert angehängt und genannt.
   - **Vorschaubild**: Jede 3MF-Datei bekommt ein Bild der Teile (schräg von vorne, ohne Hintergrund) –
     das Projekt eines je Druckplatte, wie der Slicer es in seiner Plattenliste und Dateivorschau zeigt.
   Getestet mit echten Druckern ist bisher der Weg über Bambu Studio; die Formate für die übrigen Slicer
   folgen deren Dokumentation bzw. Quellcode.

Mehr als zwei Teile: einfach weitere Verbindungen anlegen. Alles wird ausgehend vom **Basisteil**
zusammengesetzt (Basis in der Teileliste wählbar).

Bedienung: Linksklick wählt aus (nochmal klicken hebt eine Fläche wieder auf) · Rechtsklick auf einen
Punkt löscht ihn · Strg/⌘+Z nimmt die letzte Aktion zurück (Auswahl, Verbinden, Bewegen,
Ausblenden, Entfernen …), Strg/⌘+Umschalt+Z oder Strg+Y stellt sie wieder her · P/F wechselt Punkte/Flächen ·
im Gizmo G = verschieben, R = drehen, Esc = fertig · linke Maustaste dreht die Ansicht, mittlere oder
rechte verschiebt sie – stößt der Zeiger dabei an den Fensterrand, geht es unbegrenzt weiter (der Zeiger
wird bis zum Loslassen festgehalten; lässt der Browser das nicht zu, etwa in einer eingebetteten Vorschau,
dreht die Ansicht weiter, solange der Zeiger am Rand bleibt), Mausrad zoomt zum Mauszeiger · Strg/⌘ + Ziehen schiebt ein Teil (mit allem, was
daran hängt) über die Bauplatte. Nach „Verbinden" bleibt die Ansicht, wie sie ist.
**Tablet / Touch**: ein Finger dreht die Ansicht, zwei Finger zoomen und schieben, Antippen wählt aus (wie ein
Linksklick, mit etwas Spielraum für den Finger; nach einer Geste mit zwei Fingern wird nichts gewählt). **Lange
drücken** öffnet das Menü wie ein Rechtsklick – im 3D-Fenster und in der Teileliste; darüber gehen auch
Skalieren, Aussparungen, Beschriften, Bewegen usw. Oben im 3D-Fenster stehen **↶ ↷** für Rückgängig und
Wiederholen, beim Malen ersetzt „Radierer“ die Umschalttaste. Knöpfe, Felder und Menüs werden mit dem Finger
größer; ein Tablet im Hochformat zeigt das 3D-Fenster oben und die Seitenleiste darunter.
**Überhänge & Wände** (Reiter „Drucken“ oder Taste U): färbt Flächen rot, die zu flach nach unten zeigen und
Stützen brauchen (ab einstellbarem Winkel von der Senkrechten, Standard 45° – auf der Platte liegende und
innere Flächen zählen nicht), und lila, wo eine Wand dünner ist als eingestellt (Standard 0,8 mm = zwei
Bahnen bei 0,4-mm-Düse). Die Flächen stehen in der Leiste; nach dem Drehen („Auf Fläche legen", „Für den
Druck ausrichten") rechnet die Ansicht sofort neu.
**Explosionsansicht** (oben „Explosion", Taste X): zieht jede Baugruppe auseinander, damit man sieht, wie die
Teile zusammengehören – Regler unten rechts für den Abstand. Nur zum Ansehen: Lage, Export und Prüfungen
bleiben unverändert, Markierungen und Anklicken ruhen, bis sie wieder zu ist.
**Montageanleitung** (Reiter „Drucken“, unter „Exportieren“): ein Blatt zum Ausdrucken oder als PDF („Drucken / PDF“, im
Druckdialog „Als PDF speichern“) – ein Bild der sichtbaren Teile aus der Blickrichtung des 3D-Fensters, auf
Wunsch auseinandergezogen, mit nummerierten Ballons und Hinweislinien; die **Stückliste** fasst gleiche Teile
zusammen (z. B. „4× Dübel“) und nennt Maße, Filament und Gewicht; die **Schritte** kommen aus den
Verbindungen in der Reihenfolge vom Basisteil aus („Kappe auf Sockel setzen – Bohrung ⌀ 12,3 mm auf Zapfen
⌀ 12 mm“), dazu Dübel, Schnappstifte und zerteilte Stücke. Titel änderbar, das Bild auch als PNG.
**Schnittansicht** (oben „Schnitt", Taste C): schneidet alles entlang X, Y oder Z auf – Lage per Regler unten
rechts, „⇄ Seite" zeigt die andere Hälfte. Wo die Ebene durch Material geht, erscheint eine Schnittfläche in
einem dunkleren Ton des Teils (auch bei aufeinanderliegenden Teilen sauber getrennt), so lassen sich
Wandstärken, Bohrungen und Passungen ansehen. Klicken und Messen treffen nur, was sichtbar ist – z. B. eine
Bohrung im Inneren.
**Netz reparieren**: Nach dem Laden prüft D3DForge jedes Netz im Hintergrund. Offene oder verdrehte Netze
bekommen ein ⚠ in der Teileliste (und einen Hinweis in der Statuszeile); ein Klick darauf oder Rechtsklick ›
„Netz reparieren" verschweißt fast deckungsgleiche Punkte (feine Risse), entfernt doppelte Dreiecke und
Innenwände, gleicht den Umlaufsinn an, dreht nach innen zeigende Netze nach außen und schließt Löcher
(ebene Ränder sauber abgetragen, verwinkelte mit einem Fächer). Gewählte Flächen und Bemalung bleiben
erhalten. Zerschneiden und Vereinigen brauchen ein geschlossenes Netz – beim Zerschneiden gibt es dafür
direkt einen Knopf. Zusammengefallene Dreiecke ohne Fläche gelten nicht als Fehler.

**Netz vereinfachen** (Rechtsklick, auch für mehrere ausgewählte Teile): weniger Dreiecke – etwa für Scans
oder fein modellierte Figuren, die D3DForge und den Slicer bremsen. Man wählt, wie weit die Oberfläche
höchstens abweichen darf (0,01 bis 0,5 mm); die Auswahl zeigt für jede Stufe, wie viele Dreiecke übrig
bleiben. 0,05 mm sieht man im Druck nicht, für enge Passungen lieber 0,01–0,02 mm. Bemalung und gewählte
Flächen bleiben genau erhalten (ihre Ränder bleiben stehen, fein bemalte Dreiecke unverändert; teilt die
Rechnung ausnahmsweise eines, bekommt jedes Stück die Farbe an seiner Stelle). Braucht ein geschlossenes
Netz; Strg+Z macht es rückgängig.
**Toleranz-Test** (links unter den Beispielen): erzeugt eine Platte mit Löchern in steigendem Spiel (0,1 bis
0,4 mm, beschriftet) und einen Zapfen mit Griff – beides drucken, den Zapfen der Reihe nach in die Löcher
stecken; das erste Loch, in das er ohne Gewalt ganz hineingeht, nennt das passende Spiel für deinen Drucker.
Unter „Mein Spiel" gespeichert, gilt es für den gewählten Drucker und das Material unter „Filament & Kosten"
(z. B. P2S mit PLA anders als H2C mit PETG): Zerschneiden, Zerteilen und Schwalbenschwanz nehmen es als
Standard, die Passung bekommt einen Knopf „Mein Spiel" (Zapfen und Bohrungen um das doppelte Spiel, Flächen
um das einfache). Ohne eigene Messung gilt 0,2 mm.
**Aussparungen** (Rechtsklick › „Aussparung hier …" oder Taste E und auf die Stelle klicken): Tasche für einen
Scheibenmagneten (5 × 2 bis 20 × 3 mm), Loch für einen Gewindeeinsatz zum Einschmelzen (M2–M5, übliche
Maße – mit dem Datenblatt abgleichen), Schraubenloch mit Zylinder- oder Senkkopf (M2–M6), Sechskant-Tasche
für eine Mutter (mit Schraubenloch) oder eine freie Bohrung – alle Maße änderbar, mit Spiel. Die Aussparung
steht senkrecht auf der Fläche; auf runden ebenen Flächen sitzt sie in der Mitte. Eine Vorschau zeigt sie, dazu
die Materialstärke dahinter und eine Warnung, wenn sie durchgehen würde. Danach die nächste Stelle anklicken
– z. B. vier Magnete in die Ecken. Gewählte Flächen und Bemalung bleiben erhalten.
**Beschriften** (Knopf „Beschriften“ oder Taste W, Rechtsklick › „Hier beschriften …“): Text oder ein Logo aus
einer SVG-Datei (gefüllte Flächen) an der angeklickten Stelle **erhaben** aufsetzen oder **vertieft**
einlassen – Größe (Höhe der Großbuchstaben), Höhe bzw. Tiefe, Drehung und Schrift (fett/normal, mit Umlauten)
einstellbar, mehrzeilig mit Umschalt + Enter. Die Form folgt der Oberfläche, auch auf Zylindern und
Wölbungen; an senkrechten Flächen steht sie aufrecht, an waagerechten liest sie sich aus der aktuellen
Ansicht. Eine Vorschau zeigt sie vorher, mit Hinweis, wenn sie über den Rand ragt oder die Wand dahinter zu
dünn wird. Auf Wunsch bekommt sie ein eigenes Filament (als Bemalung im Slicer-Projekt – für
Mehrfarbsysteme oder mehrere Düsen). **Nummerieren** (Rechtsklick auf mehrere ausgewählte Teile › „Nummerieren …“, oder „Nummern“ in der
Leiste): prägt 1/7, 2/7 … (Muster und Startnummer frei) in die ausgewählten Teile – je auf die größte Fläche
unten (vertieft, liegt auf der Platte und ist nach dem Zusammenbau meist unsichtbar) oder oben, so groß wie
die Fläche erlaubt (höchstens die eingestellte Größe), an Bohrungen vorbei. Dübel und Schnappstifte bleiben
ohne Nummer. Alles mit Strg/⌘+Z in einem Schritt zurücknehmbar.
**Auf Druckplattengröße zerteilen**: Ist ein Teil zu groß für den gewählten Drucker, bietet die Statuszeile
„Zerteilen …" an (auch per Rechtsklick). D3DForge schlägt vor, wie viele gleich große Stücke je Achse nötig
sind, damit jedes mit Rand für die Verbinder auf die Platte passt (auch um 90° gedreht); mit − / + lässt sich
das ändern, die Schnittebenen sind rot vorgezeigt. Verbinder: Dübel (Standard – auch an senkrechten
Schnitten gut druckbar), Zapfen, Schwalbenschwanz, Schnapp (Schnappstifte, siehe Zerschneiden) oder ohne.
Danach liegen die Stücke auseinandergerückt wie im Ganzen, höhere Lagen dahinter auf der Platte, die Dübel bzw.
Schnappstifte davor; „Auf der Platte anordnen" verteilt sie.
**Zerschneiden** (Rechtsklick › „Zerschneiden …"): teilt ein Teil an der Schnittebene in zwei – etwa wenn es
nicht auf die Platte passt oder ohne Stützen druckbar werden soll. Die Ebene stellt man wie in der
Schnittansicht ein (X/Y/Z, Regler, Seite); gezeigt wird nur dieses Teil aufgeschnitten. Verbinder: ohne
(zum Kleben), **Zapfen** an der sichtbaren Hälfte mit passender Bohrung in der anderen, **Dübel**
(Bohrungen in beiden Hälften, Dübel als eigene Teile) oder **Schwalbenschwanz** (Feder quer über den
Querschnitt, 15° Flanken, die andere Hälfte wird seitlich aufgeschoben und hält ohne Kleber) oder **Schnapp**
(Bohrungen mit Hinterschnitt in beiden Hälften, geschlitzte Schnappstifte mit Widerhaken an beiden Enden als
eigene Teile – sie liegen flach auf ihrer Abflachung, drucken so ohne Stützen und rasten beim Zusammenstecken
ein; ab ⌀ 3 mm). Anzahl und ⌀ automatisch oder vorgegeben, Spiel
einstellbar (Standard 0,2 mm). Die Verbinder sitzen nur dort, wo um die Bohrung genug Wand bleibt, und
reichen nie bis auf die andere Seite durch; die Vorschau zeigt sie in der Schnittfläche. Danach liegen die
Hälften mit der Schnittfläche nach unten nebeneinander (Zapfen nach oben), Dübel stehen daneben – alles mit
Strg/⌘+Z zurücknehmbar. Das Teil muss geschlossen (wasserdicht) sein; die Bemalung bleibt (nur feine Bemalung
genau an der Schnittkante entfällt), Modifier entfallen.
**Skalieren und Spiegeln** (Rechtsklick › „Skalieren / Spiegeln …" oder Taste S über einem Teil bzw. für die
Auswahl): Größe in % oder als Breite/Tiefe/Höhe in mm (gleichmäßig), Schnellwahl 50 %, 200 %, Zoll → mm
(× 25,4) und cm → mm; Spiegeln an X, Y oder Z. Wahlweise die ganze Baugruppe oder nur ein Teil – um die
Mitte, danach wieder auf der Platte. Verbindungen bleiben erhalten: Wird alles gemeinsam umgeformt, gilt ihr
Ergebnis unverändert weiter; bei einem einzelnen Teil setzen sich die verbundenen Teile neu daran. Bemalung
und Bambu-Text skalieren mit (der Text bleibt in Bambu Studio bearbeitbar); beim Spiegeln gehen Bemalung
und Bearbeitbarkeit des Textes verloren (die Textform bleibt), D3DForge sagt das vorher.
**Mehrfachauswahl**: In der Teileliste wählt Strg/⌘+Klick einzelne Teile, Umschalt+Klick einen Bereich; im
3D-Fenster Umschalt+Klick, Strg/⌘+A alle sichtbaren, Esc hebt auf. Ausgewählte Teile haben einen Rahmen;
über der Liste (und per Rechtsklick) lassen sie sich **gruppieren** – sie bleiben, wo sie sind, und bewegen
sich danach gemeinsam (auch beim Strg+Ziehen und im Gizmo) –, ausblenden, gemeinsam einfärben oder
entfernen. „Gruppe auflösen" macht alle Teile einer Gruppe wieder einzeln beweglich.
**Genau platzieren**: Im Gizmo (✥) eines frei liegenden Teils stehen Lage (Mitte X/Y, Unterkante Z) und
„Drehen … ° um X/Y/Z" als Zahlen – Wert eingeben, Enter. Die Pfeiltasten verschieben die Auswahl (oder das
Teil im Gizmo) um 1 mm, mit Umschalt um 10 mm, mit Alt/⌥ um 0,1 mm – passend zur Blickrichtung
(rechts ist rechts im Bild); Bild↑/Bild↓ hebt und senkt (bei ausgeschalteter Druckplatte). Eine Tastenfolge
nimmt Strg/⌘+Z in einem Schritt zurück. Mehrere ausgewählte Baugruppen lassen sich über „Ausrichten …"
links/mittig/rechts bzw. vorne/hinten bündig stellen oder gleichmäßig verteilen.
**Teile verrechnen** (mehrere auswählen, dann „Verrechnen …" über der Liste oder Rechtsklick): vereinigen
(ein Körper ohne Überlappungen, die anderen gehen im Ziel auf), die anderen aus einem Teil ausschneiden
(z. B. eine Form als Aussparung; die Werkzeuge werden ausgeblendet, nicht gelöscht) oder nur die
Überschneidung behalten. Ziel ist das zuletzt angeklickte Teil; es behält Lage, Verbindungen, Bemalung und
Modifier. Alle beteiligten Netze müssen geschlossen sein.
**Rechtsklick auf ein Teil** (im 3D-Fenster oder in der Teileliste) öffnet ein Menü: ausblenden, nur dieses
Teil oder diese Baugruppe zeigen, Farbe, umbenennen, als Basis festlegen, in der Liste zeigen, mit der
angeklickten Fläche auf die Platte legen, Passung an dieser Stelle, von Hand bewegen, Baugruppe bewegen,
aus der Gruppe lösen, entfernen – auf einer gewählten Fläche zuerst „Diese Fläche abwählen". Rechtsklick
ins Leere: Ansicht anpassen, alle zeigen, anordnen, Hilfe. Rechts ziehen verschiebt weiterhin die Ansicht.
Der **Ansichtswürfel** oben rechts dreht sich mit: Fläche, Kante oder Ecke anklicken schwenkt die Ansicht
dorthin (Drehpunkt und Abstand bleiben), Ziehen am Würfel dreht sie, ⌂ zeigt die Schrägansicht. Tasten wie
im Slicer: 1 oben, 2 unten, 3 vorne, 4 hinten, 5 links, 6 rechts, 0 schräg.
Namen: Mauszeiger kurz über einem Teil halten zeigt seinen Namen, Shift+Klick wählt es aus und zeigt es in der Teileliste,
Klick auf einen Namen lässt das Teil aufleuchten, Doppelklick benennt es um. Die Seitenleiste lässt sich
am Rand breiter oder schmaler ziehen (Doppelklick: zurücksetzen).
„Alle Teile" über der Teileliste blendet alle Teile aus oder wieder ein, der Pfeil davor klappt die
Liste ein. Ausgeblendete Teile stehen in einem eigenen, einklappbaren Abschnitt; die Auswahllisten der
Verbindungen zeigen nur sichtbare Teile.
Jedes Teil lässt sich über das Pfeilkreuz in der Teileliste frei von Hand bewegen, „Gesamtes Modell bewegen" verschiebt
oder dreht das ganze zusammengesetzte Modell.

## Runde Teile

Bei Kreisen und Zylindern legt die Form die Drehung um die Achse nicht fest. D3DForge geht so vor:

1. Achsen und Mittelpunkte werden exakt ausgerichtet (Zapfen ↔ Bohrung koaxial, Kreisfläche ↔ Kreisfläche
   mittig). Das Spiel zwischen Zapfen und Bohrung wird angezeigt.
2. Die Drehung wird über den ganzen Kreis abgetastet und die Stellung mit dem besten Flächenkontakt ohne
   Durchdringung gewählt. Hat das Teil irgendetwas Unrundes (Abflachung, Nase, Lochbild, Kanten), ist die
   Stellung damit eindeutig.
3. Ist das Teil wirklich rund, sagt D3DForge das – dann ist die Drehung frei: mit dem Gizmo (Drehring um die
   Achse, Raster z. B. 15°) einstellen oder einen zusätzlichen Punkt auf beiden Teilen setzen.

## Wie viele Punkte?

| Punkte | Ergebnis |
| --- | --- |
| 1 | Teile berühren sich am Punkt, Flächen liegen aufeinander. Drehung um die Kontaktachse per Regler. |
| 2 | Lage eindeutig, wenn die Flächennormalen helfen (Standard). |
| 3+ | Am genauesten. Bei symmetrischen Anordnungen (z. B. Quadrat) warnt D3DForge, wenn mehrere Lösungen unterschiedlich aussehen – „Alternative" schaltet durch. |

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
- **Rechnen im Hintergrund**: 3MF lesen und schreiben, Projekte packen, die Suche nach markanten
  Flächen, Netz prüfen/reparieren und alle booleschen Operationen (Zerschneiden samt Vorschau, Zerteilen,
  Aussparungen, Teile verrechnen, Vereinigen beim Export, Toleranz-Test) laufen in einem Web Worker
  (`src/compute/`) – die Ansicht bleibt dabei bedienbar. Lässt der Browser keinen zu, rechnet dieselbe
  Aufgabe im Hauptthread. Nach dem Laden werden die Flächen schon im Hintergrund gesucht, damit
  „Vorschlagen" sofort antwortet.
- **CAD-Import**: `occt-import-js` (OpenCascade als WebAssembly, `src/io/step.ts`); das WebAssembly steckt
  gzip-gepackt in der Datei (`vite.config.ts`) und wird im Hintergrund-Thread entpackt und gestartet.
  OpenCascade und occt-import-js stehen unter der LGPL-2.1 (Lizenztexte in
  `node_modules/occt-import-js/dist/`); wer D3DForge weitergibt, legt diese Hinweise bei.
- **Projekte**: ZIP mit `project.json` und den Netzen als Float32-Daten, Sitzungssicherung in IndexedDB
  (`src/io/project.ts`).
- **3MF** lesen/schreiben inkl. Slicer-Metadaten (`src/io/threemf.ts`), Bemalung im Format des
  TriangleSelector von Bambu/Prusa (`src/io/paint.ts`), Darstellung von Bemalung und Modifiern per
  Stencil-Puffer (`src/decor.ts`), **Vereinigung** mit manifold-3d (`src/io/union.ts`).
- **Montageanleitung**: Bild per Render-Ziel wie das Vorschaubild, Anker per Strahl auf eine sichtbare Stelle
  jedes Teils, Ballons ohne Überdeckung verteilt, gleiche Teile über Dreiecke, Volumen und Maße erkannt
  (`src/math/sheet.ts`); gedruckt wird nur die Seite (`@media print`).
- **Beschriftung**: Schrift „Droid Sans“ (Apache-2.0, aus den three.js-Beispielen, auf lateinische Zeichen
  gekürzt: `src/fonts/`) bzw. SVG über den SVGLoader von three.js zu Umrissen (`src/text.ts`); manifold-3d
  zieht sie hoch, unterteilt sie fein und legt jeden Punkt über ein Höhenfeld aus Strahlen auf die
  Oberfläche (`src/io/emboss.ts`). Platz für Nummern: Raster der größten Fläche, Abstandsfeld, größtes
  passendes Rechteck entlang der Hauptachse (`src/math/labelspot.ts`).

## Entwicklung

```bash
npm install
npm run dev            # Entwicklungsserver
npm test               # Unit-Tests der Mathematik
npm run build          # Build nach dist/
npm run build:single   # eine einzige, eigenständige HTML-Datei: dist-single/index.html
```

## Lizenz

D3DForge steht unter der [MIT-Lizenz](LICENSE) – © 2026 Joshua Diekamp. Nutzen, ändern und weitergeben ist
erlaubt, auch kommerziell, solange der Urheberhinweis und der Lizenztext dabeibleiben.

Eingebaute Bibliotheken und Daten behalten ihre eigenen Lizenzen (auch in der Standalone-Datei und auf der
Website enthalten):

| Bestandteil | Lizenz |
| --- | --- |
| three.js | MIT |
| three-mesh-bvh | MIT |
| fflate | MIT |
| manifold-3d | Apache-2.0 |
| occt-import-js / OpenCascade | LGPL-2.1 (Lizenztexte in `node_modules/occt-import-js/dist/`) |
| Schrift Droid Sans (`src/fonts/`, aus den three.js-Beispielen) | Apache-2.0 |
