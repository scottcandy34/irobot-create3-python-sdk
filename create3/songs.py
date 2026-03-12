from .music import Note, MusicNote, Timing

class MarioTheme():
    def __init__(self):
        timing = Timing(200)
        percent = 0.3 # 30%
        self.notes = [
                    MusicNote(Note.E5, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.E5, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.REST, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.E5, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.REST, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.C5, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.E5, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.G5, timing.quarter, timing.quarter * percent),
                    MusicNote(Note.REST, timing.quarter, timing.quarter * percent),
                    MusicNote(Note.G4, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.REST, timing.quarter, timing.quarter * percent),
                    MusicNote(Note.C5, timing.dotted_quarter, timing.dotted_quarter * percent),
                    MusicNote(Note.G4, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.REST, timing.quarter, timing.quarter * percent),
                    MusicNote(Note.E4, timing.dotted_quarter, timing.dotted_quarter * percent),
                    MusicNote(Note.A4, timing.quarter, timing.quarter * percent),
                    MusicNote(Note.B4, timing.quarter, timing.quarter * percent),
                    MusicNote(Note.A4_SHARP, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.A4, timing.quarter, timing.quarter * percent),
                    MusicNote(Note.G4, timing.syncopated_eighth.long, timing.syncopated_eighth.long * percent),
                    MusicNote(Note.E5, timing.syncopated_eighth.long, timing.syncopated_eighth.long * percent),
                    MusicNote(Note.G5, timing.syncopated_eighth.long, timing.syncopated_eighth.long * percent),
                    MusicNote(Note.A5, timing.quarter, timing.quarter * percent),
                    MusicNote(Note.F5, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.G5, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.REST, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.E5, timing.quarter, timing.quarter * percent),
                    MusicNote(Note.C5, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.D5, timing.eighth, timing.eighth * percent),
                    MusicNote(Note.B4, timing.dotted_quarter, timing.dotted_quarter * percent),
                  ]