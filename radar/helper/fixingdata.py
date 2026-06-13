"""Filter für die geglättete Visualisierung der AWR1642-Punktwolke.

Die Rohdaten werden bewusst nicht verändert. Der Filter verwaltet lediglich
kurzlebige Tracks für die Darstellung im Scatter-Plot.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class _PointTrack:
    """Interner Zustand eines geglätteten Radar-Messpunkts."""

    x: float
    y: float
    confirmations: int = 1
    missing_frames: int = 0


class PointCloudLowPassFilter:
    """Glättet eine 2D-Radar-Punktwolke für die Visualisierung.

    Die Anzahl und Reihenfolge der erkannten Radarobjekte kann sich von Frame zu
    Frame ändern. Deshalb werden Messpunkte nicht anhand ihres Array-Index,
    sondern anhand ihrer räumlichen Nähe zu bereits bekannten Punkten
    zugeordnet. Zugeordnete Punkte werden mit einem exponentiellen Tiefpass
    geglättet.

    Neue Punkte werden erst nach mehreren aufeinanderfolgenden Detektionen
    sichtbar. Dadurch werden einzelne Ausreißer nicht direkt gezeichnet.

    Args:
        alpha: Gewicht des neuen Messwerts im Tiefpass. Kleine Werte glätten
            stärker, reagieren aber langsamer. Der Wertebereich ist (0, 1].
        max_match_distance: Maximale Entfernung in Metern, innerhalb der ein
            neuer Messpunkt einem bestehenden Track zugeordnet werden darf.
        min_confirmations: Anzahl passender Detektionen, bevor ein neuer Punkt
            im Plot angezeigt wird.
        max_missing_frames: Anzahl gültiger Frames, für die ein bestätigter
            Punkt bei einer kurzzeitigen fehlenden Detektion gehalten wird.
    """

    def __init__(
        self,
        alpha: float = 0.35,
        max_match_distance: float = 0.20,
        min_confirmations: int = 2,
        max_missing_frames: int = 1,
    ) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha muss im Bereich (0, 1] liegen.")
        if max_match_distance <= 0.0:
            raise ValueError("max_match_distance muss größer als 0 sein.")
        if min_confirmations < 1:
            raise ValueError("min_confirmations muss mindestens 1 sein.")
        if max_missing_frames < 0:
            raise ValueError("max_missing_frames darf nicht negativ sein.")

        self.alpha = float(alpha)
        self.max_match_distance = float(max_match_distance)
        self.min_confirmations = int(min_confirmations)
        self.max_missing_frames = int(max_missing_frames)
        self._tracks: list[_PointTrack] = []

    def reset(self) -> None:
        """Verwirft den bisherigen Filterzustand."""
        self._tracks.clear()

    def apply(self, x_values, y_values) -> tuple[np.ndarray, np.ndarray]:
        """Filtert die Messpunkte eines gültigen Radarframes.

        Args:
            x_values: X-Koordinaten des aktuellen Frames in Metern.
            y_values: Y-Koordinaten des aktuellen Frames in Metern.

        Returns:
            tuple[np.ndarray, np.ndarray]: Geglättete X- und Y-Koordinaten für
            den Scatter-Plot.
        """
        points = self._prepare_points(x_values, y_values)

        if not self._tracks:
            self._tracks = [_PointTrack(float(x), float(y)) for x, y in points]
            return self._visible_points()

        matches, unmatched_track_indices, unmatched_point_indices = self._match_points(
            points
        )

        # Bekannte Punkte mit einem exponentiellen Tiefpass glätten.
        for track_index, point_index in matches:
            track = self._tracks[track_index]
            point_x, point_y = points[point_index]

            track.x = (1.0 - self.alpha) * track.x + self.alpha * float(point_x)
            track.y = (1.0 - self.alpha) * track.y + self.alpha * float(point_y)
            track.confirmations += 1
            track.missing_frames = 0

        # Kurzzeitig fehlende Punkte noch wenige Frames halten, um Flackern zu
        # reduzieren. Danach werden sie verworfen.
        for track_index in unmatched_track_indices:
            self._tracks[track_index].missing_frames += 1

        self._tracks = [
            track
            for track in self._tracks
            if track.missing_frames <= self.max_missing_frames
        ]

        # Unbekannte Messpunkte zunächst nur als Kandidaten aufnehmen. Sichtbar
        # werden sie erst nach min_confirmations passenden Detektionen.
        for point_index in unmatched_point_indices:
            point_x, point_y = points[point_index]
            self._tracks.append(_PointTrack(float(point_x), float(point_y)))

        return self._visible_points()

    @staticmethod
    def _prepare_points(x_values, y_values) -> np.ndarray:
        """Erzeugt ein Nx2-Array und entfernt ungültige Werte."""
        x = np.asarray(x_values, dtype=float).reshape(-1)
        y = np.asarray(y_values, dtype=float).reshape(-1)

        if x.size != y.size:
            raise ValueError("x_values und y_values müssen gleich lang sein.")

        if x.size == 0:
            return np.empty((0, 2), dtype=float)

        points = np.column_stack((x, y))
        return points[np.isfinite(points).all(axis=1)]

    def _match_points(self, points: np.ndarray):
        """Ordnet neue Messpunkte bestehenden Tracks per Greedy-Matching zu."""
        num_tracks = len(self._tracks)
        num_points = len(points)

        if num_points == 0:
            return [], set(range(num_tracks)), set()

        track_points = np.array([[track.x, track.y] for track in self._tracks])
        distances = np.linalg.norm(
            track_points[:, np.newaxis, :] - points[np.newaxis, :, :], axis=2
        )

        candidates = []
        for track_index in range(num_tracks):
            for point_index in range(num_points):
                distance = float(distances[track_index, point_index])
                if distance <= self.max_match_distance:
                    candidates.append((distance, track_index, point_index))

        candidates.sort(key=lambda candidate: candidate[0])

        matches = []
        used_tracks = set()
        used_points = set()

        for _, track_index, point_index in candidates:
            if track_index in used_tracks or point_index in used_points:
                continue

            matches.append((track_index, point_index))
            used_tracks.add(track_index)
            used_points.add(point_index)

        unmatched_tracks = set(range(num_tracks)) - used_tracks
        unmatched_points = set(range(num_points)) - used_points

        return matches, unmatched_tracks, unmatched_points

    def _visible_points(self) -> tuple[np.ndarray, np.ndarray]:
        """Liefert nur bestätigte Tracks für den Plot zurück."""
        visible_tracks = [
            track
            for track in self._tracks
            if track.confirmations >= self.min_confirmations
            and track.missing_frames <= self.max_missing_frames
        ]

        if not visible_tracks:
            return np.array([], dtype=float), np.array([], dtype=float)

        x = np.array([track.x for track in visible_tracks], dtype=float)
        y = np.array([track.y for track in visible_tracks], dtype=float)
        return x, y
