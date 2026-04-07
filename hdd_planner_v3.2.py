#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HDD Trajectory Planner  v3.2
Горизонтальное направленное бурение — Планировщик траектории

Реализует каскадную систему поиска параметров для трёх режимов бурения:
  🔴 DRILLING_IN_PROGRESS  — перебор max_bend
  🟡 PRE_DRILLING_ANCHORED — перебор start_angle → max_bend
  🟢 PRE_DRILLING_NO_ANCHOR— перебор Lxx → start_angle → max_bend
"""

import math
import sys
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

VERSION = "3.2"

# ══════════════════════════════════════════════════════════════
#  ENUMS & DATACLASSES
# ══════════════════════════════════════════════════════════════

class DrillMode(Enum):
    DRILLING_IN_PROGRESS   = "БУРЕНИЕ В ПРОЦЕССЕ"
    PRE_DRILLING_ANCHORED  = "ПОДГОТОВКА С АНКЕРОВАНИЕМ"
    PRE_DRILLING_NO_ANCHOR = "ПОДГОТОВКА БЕЗ АНКЕРОВАНИЯ"


@dataclass
class TrajectoryPoint:
    rod_number: int
    x:     float   # горизонтальное расстояние от старта (м)
    depth: float   # глубина (м, >0 = под землёй)
    angle: float   # угол к горизонту (°, >0 = вниз)


@dataclass
class TrajectoryResult:
    points:       List[TrajectoryPoint]
    is_valid:     bool
    final_x:      float
    final_depth:  float
    target_x:     float
    target_depth: float
    accuracy:     float   # расстояние от конечной точки до цели (м)
    Lxx:          float
    start_angle:  float
    max_bend:     float


# ══════════════════════════════════════════════════════════════
#  CORE PLANNER
# ══════════════════════════════════════════════════════════════

class HddPlanner:
    """Ядро расчёта траектории ГНБ методом пошагового управления."""

    ROD_LENGTH = 3.0   # длина одной штанги (м)
    TOLERANCE  = 1.0   # допустимое отклонение от цели (м)

    def __init__(self):
        self.glossary = {
            "Lxx":                  "Горизонтальное расстояние до проектной точки (м)",
            "start_angle":          "Угол входа бурового инструмента (% уклона)",
            "max_bend":             "Максимальный угол изгиба штанги (° на штангу 3 м)",
            "target_depth":         "Проектная глубина бурения (м)",
            "траектория":           "Пространственная кривая движения буровой головки",
            "анкерование":          "Фиксация буровой установки к поверхности грунта",
            "штанга":               "Секция буровой колонны (~3 м)",
            "изгиб":                "Отклонение штанги от прямолинейного положения",
            "уклон":                "Отношение вертикального смещения к горизонтальному (%)",
            "валидность":           "Точность выхода буровой головки в проектную точку",
            "каскад":               "Последовательный перебор параметров при поиске решения",
            "DRILLING_IN_PROGRESS": "Бурение в процессе — Lxx и угол зафиксированы",
            "PRE_DRILLING_ANCHORED":"Подготовка с анкером — Lxx зафиксирован",
            "PRE_DRILLING_NO_ANCHOR":"Подготовка без анкера — все параметры переменные",
            "точность":             "Расстояние между конечной точкой траектории и проектной (м)",
        }

    # ── trajectory calculation ────────────────────────────────

    def calculate_trajectory(
        self,
        Lxx:             float,
        target_depth:    float,
        start_angle_pct: float,
        max_bend_pct:    float,
    ) -> TrajectoryResult:
        """
        Рассчитывает 2-D траекторию бурения.

        Параметры:
          Lxx             – горизонтальное расстояние до цели (м)
          target_depth    – глубина цели (м)
          start_angle_pct – угол входа (% уклона, напр. 20 → 11.3°)
          max_bend_pct    – макс. поворот за одну штангу (°/штангу)
        """
        # Защита от вырожденных параметров
        if max_bend_pct <= 0 or Lxx <= 0 or target_depth <= 0:
            return TrajectoryResult(
                points=[],
                is_valid=False,
                final_x=0.0,
                final_depth=0.0,
                target_x=Lxx,
                target_depth=target_depth,
                accuracy=math.hypot(Lxx, target_depth),
                Lxx=Lxx,
                start_angle=start_angle_pct,
                max_bend=max_bend_pct,
            )

        # % уклона → градусы
        start_deg    = math.degrees(math.atan(start_angle_pct / 100.0))
        # Упрощение: max_bend_pct (%) трактуется как градусы/штангу.
        # Модель даёт хорошую точность при max_bend ≤ 10°/3м (типичный диапазон ГНБ).
        max_bend_deg = max_bend_pct

        x, depth, angle = 0.0, 0.0, start_deg
        rod = 0
        points: List[TrajectoryPoint] = [
            TrajectoryPoint(rod, round(x, 2), round(depth, 2), round(angle, 2))
        ]

        # Верхняя граница штанг (запас ×3 + 100)
        max_rods = int(Lxx * 3.0 / self.ROD_LENGTH) + 100

        for _ in range(max_rods):
            dx = Lxx   - x
            dy = target_depth - depth

            if math.hypot(dx, dy) < 0.05:
                break
            if dx <= 0.0:
                break

            # Желаемый угол — прямо на цель (0–85°)
            desired = math.degrees(math.atan2(dy, dx))
            desired = max(0.0, min(85.0, desired))

            # Ограничение скорости поворота
            diff  = max(-max_bend_deg, min(max_bend_deg, desired - angle))
            angle = max(0.0, min(85.0, angle + diff))

            # Частичный шаг, если до цели меньше длины штанги
            cos_a = math.cos(math.radians(angle))
            if cos_a > 1e-9:
                max_step = dx / cos_a
            else:
                max_step = self.ROD_LENGTH
            step_len = min(self.ROD_LENGTH, max_step)

            x     += step_len * math.cos(math.radians(angle))
            depth += step_len * math.sin(math.radians(angle))
            rod   += 1
            points.append(
                TrajectoryPoint(rod, round(x, 2), round(depth, 2), round(angle, 2))
            )

            if x >= Lxx - 0.01:
                break

        accuracy = math.hypot(x - Lxx, depth - target_depth)
        is_valid = accuracy <= self.TOLERANCE

        return TrajectoryResult(
            points=points,
            is_valid=is_valid,
            final_x=round(x, 2),
            final_depth=round(depth, 2),
            target_x=Lxx,
            target_depth=target_depth,
            accuracy=round(accuracy, 3),
            Lxx=Lxx,
            start_angle=start_angle_pct,
            max_bend=max_bend_pct,
        )

    # ── display ───────────────────────────────────────────────

    def display_trajectory_table(self, result: TrajectoryResult) -> None:
        """Выводит таблицу траектории (не более 20 строк + последняя)."""
        pts = result.points
        if not pts:
            print("  (траектория пуста)")
            return

        print()
        print("  ┌─────┬──────────┬──────────┬──────────┐")
        print("  │  №  │  X (м)   │ Глуб.(м) │  Угол(°) │")
        print("  ├─────┼──────────┼──────────┼──────────┤")

        step = max(1, len(pts) // 20)
        shown: set[int] = set()
        for p in pts[::step]:
            print(f"  │ {p.rod_number:>3} │ {p.x:>8.2f} │ {p.depth:>8.2f} │ {p.angle:>8.2f} │")
            shown.add(p.rod_number)
        last = pts[-1]
        if last.rod_number not in shown:
            print(f"  │ {last.rod_number:>3} │ {last.x:>8.2f} │ {last.depth:>8.2f} │ {last.angle:>8.2f} │")

        print("  └─────┴──────────┴──────────┴──────────┘")

        status = "✅ ВАЛИДНО" if result.is_valid else "❌ НЕ ВАЛИДНО"
        print(f"\n  Цель:     X={result.target_x:.1f} м   Глуб.={result.target_depth:.1f} м")
        print(f"  Итог:     X={result.final_x:.2f} м  Глуб.={result.final_depth:.2f} м")
        print(f"  Точность: {result.accuracy:.3f} м   Статус: {status}")
        print()

    def display_glossary(self) -> None:
        print(f"\n  {'─'*58}")
        print("    ГЛОССАРИЙ ТЕРМИНОВ ГНБ")
        print(f"  {'─'*58}")
        for term, defn in self.glossary.items():
            print(f"  {term:<28} – {defn}")
        print(f"  {'─'*58}\n")


# ══════════════════════════════════════════════════════════════
#  MODE-SPECIFIC SEARCHER  (cascade logic)
# ══════════════════════════════════════════════════════════════

class ModeSpecificSearcher:
    """
    Реализует каскадную логику поиска параметров для трёх режимов.

    Правила перехода между уровнями:
      • Режим 1 (max_bend):          НЕТ → пробуем следующее значение
      • Режим 2 уровень 1 (angle):   НЕТ → переходим на уровень 2
      • Режим 3 уровень 1 (Lxx):     НЕТ → переходим на уровень 2
      • Режим 3 уровень 2 (angle):   НЕТ → переходим на уровень 3
      • Последний уровень (max_bend): решение принимается автоматически
    """

    MAX_BEND_LIMIT = 10.0   # абсолютный максимум max_bend (%)
    BEND_STEP      = 0.5    # шаг перебора max_bend
    ANGLE_STEP     = 1.0    # шаг перебора start_angle

    def __init__(
        self,
        planner:      HddPlanner,
        mode:         DrillMode,
        Lxx:          float,
        target_depth: float,
        start_angle:  float,
        max_bend:     float,
    ):
        self.planner      = planner
        self.mode         = mode
        self.Lxx          = Lxx
        self.target_depth = target_depth
        self.start_angle  = start_angle
        self.max_bend     = max_bend

    # ── helpers ───────────────────────────────────────────────

    @staticmethod
    def _progress_bar(current: int, total: int, width: int = 33) -> str:
        filled = int(width * current / total) if total > 0 else 0
        bar = "█" * filled + "░" * (width - filled)
        pct = 100.0 * current / total if total > 0 else 0.0
        return f"   {bar} {pct:5.1f}%"

    def _ask_user_approval(
        self,
        param_name: str,
        old_value,
        new_value,
        question: Optional[str] = None,
    ) -> bool:
        """
        Интерактивный диалог согласования изменения параметра.

        Возвращает True если пользователь согласен, False если нет.
        """
        print()
        print("  ┌─────────────────────────────────────────────┐")
        print(f"  │  ❓ Изменение параметра: {param_name:<20}│")
        print(f"  │     Было:   {str(old_value):<33}│")
        print(f"  │     Станет: {str(new_value):<33}│")
        print("  └─────────────────────────────────────────────┘")

        prompt = f"  ❓ {question if question else f'Применить {param_name} = {new_value}?'} (да/нет): "

        while True:
            try:
                answer = input(prompt).strip().lower()
            except (EOFError, KeyboardInterrupt):
                print("\n  (прерывание ввода — считается как «нет»)")
                return False

            if answer in ("да", "yes", "y", "д", "1"):
                print("  🎉 РЕШЕНИЕ ПРИНЯТО!\n")
                return True
            if answer in ("нет", "no", "n", "н", "0"):
                print("  ⚠️  Перехожу на следующий уровень...\n")
                return False
            print("  Введите «да» или «нет».")

    # ── range helpers ─────────────────────────────────────────

    def _lxx_range(self) -> List[float]:
        """
        Адаптивный список значений Lxx для перебора.

        Правила (из ТЗ v3.2):
          Lxx ≤  50 м → шаг  5 м (10 … 50)
          Lxx ≤ 150 м → шаг 10 м (20 … 150)
          Lxx >  150 м → шаг 20 м (50 … 500)
        """
        base = self.Lxx
        if base <= 50:
            step, lo, hi = 5, 10, 55
        elif base <= 150:
            step, lo, hi = 10, 20, 160
        else:
            step, lo, hi = 20, 50, 520

        values = [v for v in range(lo, hi, step) if v != int(base)]
        # Сортируем по близости к исходному значению
        values.sort(key=lambda v: abs(v - base))
        return [float(v) for v in values]

    def _angle_range(self, base: float) -> List[float]:
        """start_angle ∈ [base−5%, base+10%] с шагом ANGLE_STEP (без base)."""
        lo = max(1.0, base - 5.0)
        hi = base + 10.0
        result = []
        v = lo
        while v <= hi + 1e-9:
            r = round(v, 1)
            if abs(r - base) > 1e-9:
                result.append(r)
            v += self.ANGLE_STEP
        return result

    def _bend_range(self, base: float) -> List[float]:
        """max_bend от base+BEND_STEP до MAX_BEND_LIMIT шагом BEND_STEP."""
        result = []
        v = base + self.BEND_STEP
        while v <= self.MAX_BEND_LIMIT + 1e-9:
            result.append(round(v, 1))
            v += self.BEND_STEP
        return result

    # ── MODE 1: DRILLING_IN_PROGRESS ──────────────────────────

    def search_drilling_in_progress(self) -> Optional[TrajectoryResult]:
        """
        🔴 РЕЖИМ: БУРЕНИЕ В ПРОЦЕССЕ
        Зафиксировано: Lxx, start_angle
        Перебираем max_bend от текущего+шаг до 10%.
        При отказе пользователя — пробуем следующее значение.
        """
        print(f"\n{'═'*60}")
        print("🔍 Поиск решения в режиме БУРЕНИЕ В ПРОЦЕССЕ...")
        print("   Перебираю: max_bend")
        print(f"{'═'*60}\n")

        bend_values = self._bend_range(self.max_bend)
        total = len(bend_values)

        if total == 0:
            print("❌ max_bend уже на максимуме (10%). Решение невозможно.")
            return None

        print("📍 УРОВЕНЬ 1️⃣: ПЕРЕБИРАЮ max_bend\n")

        for idx, mb in enumerate(bend_values):
            sys.stdout.write(
                f"\r{self._progress_bar(idx + 1, total)} | max_bend: {mb}%  "
            )
            sys.stdout.flush()

            result = self.planner.calculate_trajectory(
                self.Lxx, self.target_depth, self.start_angle, mb
            )

            if result.is_valid:
                print(f"\n\n✅ НАЙДЕНО РЕШЕНИЕ при max_bend = {mb}%")
                self.planner.display_trajectory_table(result)

                approved = self._ask_user_approval(
                    "max_bend",
                    f"{self.max_bend}%",
                    f"{mb}%",
                    f"Увеличить max_bend с {self.max_bend}% до {mb}%?",
                )
                if approved:
                    return result
                # НЕТ → пробуем дальше (следующее значение max_bend)
                print("⚠️  Пробую следующее значение max_bend...\n")

        print("\n❌ Решение невозможно: даже при max_bend=10% цель не достигнута.")
        return None

    # ── MODE 2: PRE_DRILLING_ANCHORED ─────────────────────────

    def search_pre_drilling_anchored(self) -> Optional[TrajectoryResult]:
        """
        🟡 РЕЖИМ: ПОДГОТОВКА С АНКЕРОВАНИЕМ
        Зафиксировано: Lxx
        Каскад: start_angle → max_bend
        """
        print(f"\n{'═'*60}")
        print("🔍 Поиск решения в режиме ПОДГОТОВКА С АНКЕРОВАНИЕМ...")
        print("   Перебираю: start_angle → max_bend")
        print(f"{'═'*60}\n")

        # ── УРОВЕНЬ 1: start_angle ────────────────────────────
        print("📍 УРОВЕНЬ 1️⃣: ПЕРЕБИРАЮ start_angle\n")

        angle_values = self._angle_range(self.start_angle)
        total = len(angle_values)
        found_valid_at_level1 = False

        for idx, sa in enumerate(angle_values):
            sys.stdout.write(
                f"\r{self._progress_bar(idx + 1, total)} | start_angle: {sa}%  "
            )
            sys.stdout.flush()

            result = self.planner.calculate_trajectory(
                self.Lxx, self.target_depth, sa, self.max_bend
            )

            if result.is_valid:
                found_valid_at_level1 = True
                print(f"\n\n✅ НАЙДЕНО РЕШЕНИЕ при start_angle = {sa}%")
                self.planner.display_trajectory_table(result)

                approved = self._ask_user_approval(
                    "start_angle",
                    f"{self.start_angle}%",
                    f"{sa}%",
                    f"Возможно изменить угол входа с {self.start_angle}% на {sa}%?",
                )
                if approved:
                    return result
                # НЕТ → сразу переходим на УРОВЕНЬ 2
                break

        if not found_valid_at_level1:
            print("\n⚠️  Изменение start_angle не привело к решению. Перехожу на УРОВЕНЬ 2...\n")
        else:
            print("⚠️  Перехожу на УРОВЕНЬ 2...\n")

        # ── УРОВЕНЬ 2: max_bend ───────────────────────────────
        print("📍 УРОВЕНЬ 2️⃣: ПЕРЕБИРАЮ max_bend\n")

        bend_values = self._bend_range(self.max_bend)
        total = len(bend_values)

        for idx, mb in enumerate(bend_values):
            sys.stdout.write(
                f"\r{self._progress_bar(idx + 1, total)} | max_bend: {mb}%  "
            )
            sys.stdout.flush()

            result = self.planner.calculate_trajectory(
                self.Lxx, self.target_depth, self.start_angle, mb
            )

            if result.is_valid:
                print(f"\n\n✅ НАЙДЕНО РЕШЕНИЕ при max_bend = {mb}%")
                self.planner.display_trajectory_table(result)
                return result

        print("\n❌ Решение невозможно при любых параметрах.")
        return None

    # ── MODE 3: PRE_DRILLING_NO_ANCHOR ────────────────────────

    def search_pre_drilling_no_anchor(self) -> Optional[TrajectoryResult]:
        """
        🟢 РЕЖИМ: ПОДГОТОВКА БЕЗ АНКЕРОВАНИЯ
        Каскад: Lxx → start_angle → max_bend
        """
        print(f"\n{'═'*60}")
        print("🔍 Поиск решения в режиме ПОДГОТОВКА БЕЗ АНКЕРОВАНИЯ...")
        print("   Перебираю: Lxx, start_angle, max_bend")
        print(f"{'═'*60}\n")

        # ── УРОВЕНЬ 1: Lxx ────────────────────────────────────
        print("📍 УРОВЕНЬ 1️⃣: ПЕРЕБИРАЮ Lxx\n")

        lxx_values = self._lxx_range()
        total = len(lxx_values)
        found_valid_at_level1 = False

        for idx, lx in enumerate(lxx_values):
            sys.stdout.write(
                f"\r{self._progress_bar(idx + 1, total)} | Lxx: {lx:.0f}м  "
            )
            sys.stdout.flush()

            result = self.planner.calculate_trajectory(
                lx, self.target_depth, self.start_angle, self.max_bend
            )

            if result.is_valid:
                found_valid_at_level1 = True
                print(f"\n\n✅ НАЙДЕНО РЕШЕНИЕ при Lxx = {lx:.0f}м")
                self.planner.display_trajectory_table(result)

                approved = self._ask_user_approval(
                    "Lxx",
                    f"{self.Lxx:.0f}м",
                    f"{lx:.0f}м",
                    f"Смещение на {lx:.0f}м возможно?",
                )
                if approved:
                    return result
                # НЕТ → сразу переходим на УРОВЕНЬ 2
                break

        if not found_valid_at_level1:
            print("\n⚠️  Изменение Lxx не привело к решению. Перехожу на УРОВЕНЬ 2...\n")
        else:
            print("⚠️  Перехожу на УРОВЕНЬ 2...\n")

        # ── УРОВЕНЬ 2: start_angle ────────────────────────────
        print("📍 УРОВЕНЬ 2️⃣: ПЕРЕБИРАЮ start_angle\n")

        angle_values = self._angle_range(self.start_angle)
        total = len(angle_values)
        found_valid_at_level2 = False

        for idx, sa in enumerate(angle_values):
            sys.stdout.write(
                f"\r{self._progress_bar(idx + 1, total)} | start_angle: {sa}%  "
            )
            sys.stdout.flush()

            result = self.planner.calculate_trajectory(
                self.Lxx, self.target_depth, sa, self.max_bend
            )

            if result.is_valid:
                found_valid_at_level2 = True
                print(f"\n\n✅ НАЙДЕНО РЕШЕНИЕ при start_angle = {sa}%")
                self.planner.display_trajectory_table(result)

                approved = self._ask_user_approval(
                    "start_angle",
                    f"{self.start_angle}%",
                    f"{sa}%",
                    f"Возможно изменить угол входа с {self.start_angle}% на {sa}%?",
                )
                if approved:
                    return result
                # НЕТ → сразу переходим на УРОВЕНЬ 3
                break

        if not found_valid_at_level2:
            print("\n⚠️  Изменение start_angle не привело к решению. Перехожу на УРОВЕНЬ 3...\n")
        else:
            print("⚠️  Перехожу на УРОВЕНЬ 3...\n")

        # ── УРОВЕНЬ 3: max_bend ───────────────────────────────
        print("📍 УРОВЕНЬ 3️⃣: ПЕРЕБИРАЮ max_bend\n")

        bend_values = self._bend_range(self.max_bend)
        total = len(bend_values)

        for idx, mb in enumerate(bend_values):
            sys.stdout.write(
                f"\r{self._progress_bar(idx + 1, total)} | max_bend: {mb}%  "
            )
            sys.stdout.flush()

            result = self.planner.calculate_trajectory(
                self.Lxx, self.target_depth, self.start_angle, mb
            )

            if result.is_valid:
                print(f"\n\n✅ НАЙДЕНО РЕШЕНИЕ при max_bend = {mb}%")
                self.planner.display_trajectory_table(result)
                return result

        print("\n❌ Решение невозможно: все три уровня каскада исчерпаны.")
        return None


# ══════════════════════════════════════════════════════════════
#  INTERACTIVE CLI
# ══════════════════════════════════════════════════════════════

def _input_float(prompt: str, lo: float = None, hi: float = None) -> float:
    """Ввод числа с плавающей точкой с проверкой диапазона."""
    while True:
        try:
            val = float(input(prompt))
        except ValueError:
            print("  ⚠️  Введите число.")
            continue
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)

        if lo is not None and val < lo:
            print(f"  ⚠️  Значение должно быть ≥ {lo}")
            continue
        if hi is not None and val > hi:
            print(f"  ⚠️  Значение должно быть ≤ {hi}")
            continue
        return val


def _select_mode() -> DrillMode:
    """Интерактивный выбор режима бурения."""
    print("\n  Выберите режим бурения:")
    modes = list(DrillMode)
    icons = ["🔴", "🟡", "🟢"]
    for i, (m, icon) in enumerate(zip(modes, icons), 1):
        print(f"  {i}. {icon} {m.value}")
    while True:
        try:
            choice = int(input("  Ваш выбор (1-3): "))
            if 1 <= choice <= 3:
                return modes[choice - 1]
        except (ValueError, EOFError, KeyboardInterrupt):
            pass
        print("  ⚠️  Введите 1, 2 или 3.")


def _read_common_params():
    """Считывает общие параметры: Lxx, глубину, угол входа, макс. изгиб."""
    Lxx         = _input_float("  Lxx (м, > 0): ",          lo=1.0)
    depth       = _input_float("  Глубина (м, > 0): ",       lo=0.1)
    start_angle = _input_float("  Угол входа (%, 1-90): ",   lo=1.0, hi=90.0)
    max_bend    = _input_float(
        f"  Макс. изгиб (%, {ModeSpecificSearcher.BEND_STEP}-"
        f"{ModeSpecificSearcher.MAX_BEND_LIMIT}): ",
        lo=ModeSpecificSearcher.BEND_STEP,
        hi=ModeSpecificSearcher.MAX_BEND_LIMIT,
    )
    return Lxx, depth, start_angle, max_bend


def main():
    print(f"\n{'═'*60}")
    print(f"  HDD Trajectory Planner  v{VERSION}")
    print(f"  Горизонтальное направленное бурение")
    print(f"{'═'*60}\n")

    planner = HddPlanner()

    menu = (
        "\n  ГЛАВНОЕ МЕНЮ\n"
        "  1. Рассчитать траекторию\n"
        "  2. Поиск параметров (каскад)\n"
        "  3. Глоссарий\n"
        "  4. Выход\n"
    )

    while True:
        print(menu)
        try:
            choice = input("  Выбор: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n  До свидания!")
            sys.exit(0)

        # ── 1. Прямой расчёт ─────────────────────────────────
        if choice == "1":
            print("\n  === РАСЧЁТ ТРАЕКТОРИИ ===")
            Lxx, depth, start_angle, max_bend = _read_common_params()
            result = planner.calculate_trajectory(Lxx, depth, start_angle, max_bend)
            planner.display_trajectory_table(result)

        # ── 2. Каскадный поиск ───────────────────────────────
        elif choice == "2":
            print("\n  === ПОИСК ПАРАМЕТРОВ (КАСКАД) ===")
            mode = _select_mode()
            Lxx, depth, start_angle, max_bend = _read_common_params()

            searcher = ModeSpecificSearcher(
                planner, mode, Lxx, depth, start_angle, max_bend
            )

            if mode == DrillMode.DRILLING_IN_PROGRESS:
                result = searcher.search_drilling_in_progress()
            elif mode == DrillMode.PRE_DRILLING_ANCHORED:
                result = searcher.search_pre_drilling_anchored()
            else:
                result = searcher.search_pre_drilling_no_anchor()

            if result is not None:
                print(f"\n  ✅ Финальные параметры:")
                print(f"     Lxx         = {result.Lxx} м")
                print(f"     start_angle = {result.start_angle} %")
                print(f"     max_bend    = {result.max_bend} %")
                print(f"     Точность    = {result.accuracy} м")
            else:
                print("\n  ❌ Подходящее решение не найдено.")

        # ── 3. Глоссарий ─────────────────────────────────────
        elif choice == "3":
            planner.display_glossary()

        # ── 4. Выход ─────────────────────────────────────────
        elif choice == "4":
            print("\n  До свидания!")
            sys.exit(0)

        else:
            print("  ⚠️  Неверный ввод. Введите 1, 2, 3 или 4.")


if __name__ == "__main__":
    main()
