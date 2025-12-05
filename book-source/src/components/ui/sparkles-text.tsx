import { cn } from "@site/src/lib/utils";
import { useEffect, useState } from "react";

interface Sparkle {
  id: number;
  x: number;
  y: number;
  size: number;
  delay: number;
  color: string;
}

interface SparklesTextProps {
  children: React.ReactNode;
  className?: string;
  sparkleCount?: number;
  colors?: { first: string; second: string };
}

const generateSparkle = (colors: { first: string; second: string }): Sparkle => ({
  id: Math.random(),
  x: Math.random() * 100,
  y: Math.random() * 100,
  size: Math.random() * 10 + 6,
  delay: Math.random() * 2,
  color: Math.random() > 0.5 ? colors.first : colors.second,
});

export const SparklesText = ({
  children,
  className,
  sparkleCount = 12,
  colors = { first: "hsl(174, 72%, 56%)", second: "hsl(82, 84%, 55%)" },
}: SparklesTextProps) => {
  const [sparkles, setSparkles] = useState<Sparkle[]>([]);

  useEffect(() => {
    const initialSparkles = Array.from({ length: sparkleCount }, () =>
      generateSparkle(colors)
    );
    setSparkles(initialSparkles);

    const interval = setInterval(() => {
      setSparkles((prev) =>
        prev.map((sparkle) =>
          Math.random() > 0.7 ? generateSparkle(colors) : sparkle
        )
      );
    }, 800);

    return () => clearInterval(interval);
  }, [sparkleCount, colors.first, colors.second]);

  return (
    <span className={cn("relative inline-block", className)}>
      {sparkles.map((sparkle) => (
        <svg
          key={sparkle.id}
          className="pointer-events-none absolute animate-sparkle"
          style={{
            left: `${sparkle.x}%`,
            top: `${sparkle.y}%`,
            width: sparkle.size,
            height: sparkle.size,
            animationDelay: `${sparkle.delay}s`,
          }}
          viewBox="0 0 160 160"
          fill="none"
        >
          <path
            d="M80 0C80 0 84.2846 41.2925 101.496 58.504C118.707 75.7154 160 80 160 80C160 80 118.707 84.2846 101.496 101.496C84.2846 118.707 80 160 80 160C80 160 75.7154 118.707 58.504 101.496C41.2925 84.2846 0 80 0 80C0 80 41.2925 75.7154 58.504 58.504C75.7154 41.2925 80 0 80 0Z"
            fill={sparkle.color}
          />
        </svg>
      ))}
      <span className="relative z-10">{children}</span>
    </span>
  );
};
