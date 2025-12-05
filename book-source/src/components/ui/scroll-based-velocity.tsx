import { cn } from "@site/src/lib/utils";

interface ScrollVelocityProps {
  text: string;
  className?: string;
  velocity?: number;
}

export const ScrollVelocity = ({
  text,
  className,
  velocity = 20,
}: ScrollVelocityProps) => {
  const repeatedText = Array(4).fill(text).join(" • ");

  return (
    <div className={cn("relative w-full overflow-hidden py-8", className)}>
      <div
        className="flex whitespace-nowrap animate-scroll-velocity"
        style={{
          animationDuration: `${velocity}s`,
        }}
      >
        <span className="mr-8 text-6xl font-bold uppercase tracking-wider text-transparent bg-clip-text bg-linear-to-r from-primary via-secondary to-primary md:text-8xl">
          {repeatedText}
        </span>
        <span className="mr-8 text-6xl font-bold uppercase tracking-wider text-transparent bg-clip-text bg-linear-to-r from-primary via-secondary to-primary md:text-8xl">
          {repeatedText}
        </span>
      </div>
    </div>
  );
};
