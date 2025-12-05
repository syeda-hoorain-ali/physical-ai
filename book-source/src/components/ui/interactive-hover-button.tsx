import { cn } from "@site/src/lib/utils";
import { ArrowRight } from "lucide-react";

interface InteractiveHoverButtonProps {
  children: React.ReactNode;
  className?: string;
  onClick?: () => void;
}

export const InteractiveHoverButton = ({
  children,
  className,
  onClick,
}: InteractiveHoverButtonProps) => {
  return (
    <button
      onClick={onClick}
      className={cn(
        "group relative inline-flex items-center gap-2 overflow-hidden rounded-full px-8 py-4 font-semibold transition-all duration-300",
        "bg-linear-to-r from-primary to-secondary text-primary-foreground",
        "hover:shadow-lg hover:shadow-primary/30",
        "animate-pulse-glow",
        className
      )}
    >
      <span className="relative z-10 transition-transform duration-300 group-hover:-translate-x-1">
        {children}
      </span>
      <ArrowRight className="relative z-10 h-5 w-5 transition-all duration-300 group-hover:translate-x-1" />
      
      {/* Hover background effect */}
      <div className="absolute inset-0 z-0 bg-linear-to-r from-secondary to-primary opacity-0 transition-opacity duration-300 group-hover:opacity-100" />
      
      {/* Shine effect */}
      <div className="absolute inset-0 z-0 -translate-x-full bg-linear-to-r from-transparent via-white/20 to-transparent transition-transform duration-500 group-hover:translate-x-full" />
    </button>
  );
};
