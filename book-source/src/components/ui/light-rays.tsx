import { cn } from "@site/src/lib/utils";

interface LightRaysProps {
  className?: string;
  rayCount?: number;
}

export const LightRays = ({ className, rayCount = 8 }: LightRaysProps) => {
  const rays = Array.from({ length: rayCount }, (_, i) => ({
    id: i,
    left: `${(i / rayCount) * 100}%`,
    delay: i * 0.3,
    width: Math.random() * 60 + 40,
    opacity: Math.random() * 0.3 + 0.2,
  }));

  return (
    <div
      className={cn(
        "pointer-events-none absolute inset-0 overflow-hidden",
        className
      )}
    >
      {rays.map((ray) => (
        <div
          key={ray.id}
          className="absolute top-0 h-full animate-light-ray"
          style={{
            left: ray.left,
            width: `${ray.width}px`,
            animationDelay: `${ray.delay}s`,
            background: `linear-gradient(
              180deg,
              transparent 0%,
              hsl(174, 72%, 56%, ${ray.opacity}) 20%,
              hsl(82, 84%, 55%, ${ray.opacity * 0.8}) 50%,
              hsl(174, 72%, 56%, ${ray.opacity * 0.5}) 80%,
              transparent 100%
            )`,
            filter: "blur(20px)",
            transform: "skewX(-15deg)",
          }}
        />
      ))}
      {/* Central glow */}
      <div
        className="absolute left-1/2 top-0 h-full w-96 -translate-x-1/2 opacity-40"
        style={{
          background:
            "radial-gradient(ellipse at 50% 0%, hsl(174, 72%, 56%, 0.4), transparent 70%)",
        }}
      />
    </div>
  );
};
