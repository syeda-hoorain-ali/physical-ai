import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import Translate from '@docusaurus/Translate';
import type { Props } from '@theme/NotFound/Content';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';

export default function NotFoundContent({ className }: Props): ReactNode {
  return (
    <div className={clsx("flex min-h-screen items-center justify-center bg-muted", className)}>
      <div className="text-center">
        <Heading as="h1" className="mb-4 text-4xl font-bold">
          <Translate
            id="theme.NotFound.title"
            description="The title of the 404 page">
            404
          </Translate>
        </Heading>
        <p className="mb-4 text-xl text-muted-foreground">
          <Translate
            id="theme.NotFound.p1"
            description="The first paragraph of the 404 page">
            We could not find what you were looking for.
          </Translate>
        </p>
        <p className="mb-4 text-xl text-muted-foreground">
          <Translate
            id="theme.NotFound.p2"
            description="The 2nd paragraph of the 404 page">
            Please contact the owner of the site that linked you to the
            original URL and let them know their link is broken.
          </Translate>
        </p>
        <Link href="/" className="text-primary underline hover:text-primary/90">
          <Translate
            id="theme.NotFound.a"
            description="Link to home page">
            Return to Home
          </Translate>
        </Link>
      </div>
    </div>
  );
}
